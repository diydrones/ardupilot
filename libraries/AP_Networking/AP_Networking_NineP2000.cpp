#include "AP_Networking_Config.h"

#if AP_NETWORKING_FILESYSTEM_ENABLED

#include "AP_Networking.h"
#include <AP_HAL/utility/Socket.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Math/AP_Math.h>
#include <errno.h>
#include <AP_Filesystem/AP_Filesystem.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo NineP2000::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable 9P2000 client
    // @Description: 9P2000 client allows file access to enteral server over a TCP connection.
    // @Values: 0:Disabled, 1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, NineP2000, enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Group: IP
    // @Path: AP_Networking_address.cpp
    // @RebootRequired: True
    AP_SUBGROUPINFO(ip, "IP", 2, NineP2000, AP_Networking_IPV4),

    // @Param: PORT
    // @DisplayName: Port number
    // @Description: Port number
    // @Range: 0 65535
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("PORT", 3, NineP2000, port, 0),

    AP_GROUPEND
};

/*
  initialise mapped network ports
 */
void NineP2000::init()
{
    sock = NEW_NOTHROW SocketAPM(false);
    if (sock != nullptr) {
        sock->set_blocking(true);

        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&NineP2000::loop, void), "9P2000", 1024, AP_HAL::Scheduler::PRIORITY_STORAGE, 0)) {
            AP_BoardConfig::allocation_error("9P2000 thread");
        }
    }

}

void NineP2000::loop()
{
    AP::network().startup_wait();

    bool active = false;
    while (true) {
        if (!active) {
            hal.scheduler->delay_microseconds(100);
        }
        if (sock == nullptr) {
            sock = NEW_NOTHROW SocketAPM(false);
            if (sock == nullptr) {
                continue;
            }
            sock->set_blocking(true);
            connected = false;
        }
        if (!connected) {
            const char *dest = ip.get_str();
            connected = sock->connect(dest, port.get());
            if (!connected) {
                delete sock;
                sock = nullptr;
                // don't try and connect too fast
                hal.scheduler->delay(100);
                continue;
            }
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "9P2000: connected to %s:%u", dest, unsigned(port.get()));
            sock->set_blocking(false);

            // Clear message and file tracking
            memset(&receive, 0, sizeof(receive));
            memset(&fileIds, 0, sizeof(fileIds));

            // Restart connection process
            request_version();

        }
        active = update();
    }
}

// Return true if connected and mounted
bool NineP2000::mounted()
{
    return connected && (state == State::Mounted);
}

// Deal with incoming data
bool NineP2000::update()
{
    const auto len = sock->recv(receive.buffer, sizeof(receive.buffer), 0);
    if (len == 0) {
        // Zero means disconnected
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "9P2000: closed connection");
        delete sock;
        sock = nullptr;
        return false;
    }
    if (len < 0) {
        // Could return -1 if there is no pending data
        return false;
    }

    parse(len);
    return true;
}

void NineP2000::parse(const uint32_t len)
{
    // Need at least the length the of the massage
    if (len < receive.header.length) {
        return;
    }

    // Ignore any messages longer than the negotiated max length
    if (receive.header.length > bufferLen) {
        return;
    }

    // Use semaphore for thread safety
    WITH_SEMAPHORE(request_sem);

    // Deal with each message type
    const Type msg_type = (Type)receive.header.type;
    switch (msg_type) {
        case Type::Rversion: {
            if (state != State::Version) {
                // Should only get a version response if we asked for one
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            }
            handle_version();
            break;
        }

        case Type::Rattach: {
            if (state != State::Attach) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            }
            handle_attach();
            break;
        }

        // Auth and flush are not supported
        case Type::Rauth:
        case Type::Rflush:
            break;

        // Clear tag from clunk
        case Type::Rclunk: {
            // Note that there is no timeout, so we could leak a tag and a file id
            // Clear the tag and file ID so they can be used again
            const uint16_t tag = receive.header.tag;

            // Check tag is valid
            if ((tag >= ARRAY_SIZE(request)) || !request[tag].pending) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                return;
            }

            // Check that the type matches what is expected
            if (request[tag].expectedType != Type::Rclunk) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                return;
            }

            clear_file_id(request[tag].clunk.fileId);
            clear_tag(tag);
            break;
        }

        // Stash result for callback
        case Type::Rerror:
        case Type::Rwalk:
        case Type::Ropen:
        case Type::Rcreate:
        case Type::Rread:
        case Type::Rwrite:
        case Type::Rremove:
        case Type::Rstat:
        case Type::Rwstat: {
            // Should be mounted before responses start turning up
            if (state != State::Mounted) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                return;
            }

            // Check tag is valid
            const uint16_t tag = receive.header.tag;
            if ((tag >= ARRAY_SIZE(request)) || !request[tag].pending) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                return;
            }

            // Check that the type matches what is expected (unexpected errors are allowed)
            if ((msg_type != Type::Rerror) && (request[tag].expectedType != msg_type)) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                return;
            }

            switch (msg_type) {
                case Type::Rerror:
                    handle_error(request[tag]);
                    break;

                case Type::Rwalk:
                    handle_rwalk(request[tag]);
                    break;

                case Type::Ropen:
                    // If we got a valid response the open worked
                    request[tag].open.result = receive.header.length == (sizeof(Message::header) + sizeof(Message::Ropen));
                    break;

                case Type::Rcreate:
                    request[tag].create.result = receive.header.length == (sizeof(Message::header) + sizeof(Message::Rcreate));
                    break;

                case Type::Rread: {
                    if (request[tag].read.is_dir) {
                        handle_dir_Rread(request[tag]);
                    } else {
                        handle_file_Rread(request[tag]);
                    }
                    break;
                }

                case Type::Rwrite: {
                    if (receive.header.length == (sizeof(Message::header) + sizeof(Message::Rwrite))) {
                        request[tag].write.count = receive.Rwrite.count;
                    }
                    break;
                }

                case Type::Rremove:
                    request[tag].remove.result = receive.header.length == sizeof(Message::header);
                    break;

                case Type::Rstat:
                    handle_Rstat(request[tag]);
                    break;

                case Type::Rwstat:
                    request[tag].rwstat.result = receive.header.length == sizeof(Message::header);
                    break;

                default:
                    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                    break;
            }

            // No longer pending
            request[tag].pending = false;
            break;
        }

        // Not expecting to receive any requests
        case Type::Tversion:
        case Type::Tauth:
        case Type::Tattach:
        case Type::Tflush:
        case Type::Twalk:
        case Type::Topen:
        case Type::Tcreate:
        case Type::Tread:
        case Type::Twrite:
        case Type::Tclunk:
        case Type::Tremove:
        case Type::Tstat:
        case Type::Twstat:
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
    }

    // Parsed full length of message
    if (receive.header.length >= len) {
        return;
    }

    // Try parsing the remainder
    const uint16_t remaining = len - receive.header.length;
    memmove(&receive.buffer[0], &receive.buffer[receive.header.length], remaining);

    parse(remaining);
}

// Add a string to the end of a message
bool NineP2000::add_string(Message &msg, const char *str) const
{
    const size_t offset = msg.header.length;

    const uint16_t len = strlen(str);

    if ((offset + sizeof(len) + len) > MIN(bufferLen, sizeof(Message))) {
        // This would be a huge file name!
        return false;
    }

    // Add string length and string content;
    memcpy(&msg.buffer[offset], &len, sizeof(len));
    strncpy_noterm((char*)&msg.buffer[offset + sizeof(len)], str, len);

    msg.header.length += sizeof(len) + len;

    return true;
}

// Request version and message size
void NineP2000::request_version()
{
    state = State::Version;
    bufferLen = 32; // Assume a minimum message length

    send.header.type = (uint8_t)Type::Tversion;
    send.header.tag = NOTAG;
    send.header.length = sizeof(Message::header) + sizeof(Message::Tversion);
    send.Tversion.msize = sizeof(Message);

    if (!add_string(send, "9P2000")) {
        // This should never fail, bufferLen is 32, there is room for the string
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }

    sock->send(send.buffer, send.header.length);
}

// handle version response
void NineP2000::handle_version()
{
    // Should be at least the min length, string increases total length
    const uint32_t version_len = sizeof(Message::header) + sizeof(Message::Rversion);
    if (receive.header.length < version_len) {
        return;
    }

    if (receive.header.tag != NOTAG) {
        // tag should always be NOTAG, throw a error?
        return;
    }

    // Message length should be equal to or less than the value requested
    if (receive.Rversion.msize > sizeof(Message)) {
        return;
    }

    // Make sure size is sufficient to fit all fixed length messages
    // Max len should be 55, with header of 7 we need at least 62 bytes
    size_t max_len = sizeof(Message::Tversion) + sizeof(uint16_t);
    max_len = MAX(max_len, sizeof(Message::Rversion));
    max_len = MAX(max_len, sizeof(Message::Tattach) + 2 * sizeof(uint16_t));
    max_len = MAX(max_len, sizeof(Message::Rattach));
    max_len = MAX(max_len, sizeof(Message::Tclunk));
    max_len = MAX(max_len, sizeof(Message::Rerror));
    max_len = MAX(max_len, sizeof(Message::Topen));
    max_len = MAX(max_len, sizeof(Message::Ropen));
    max_len = MAX(max_len, sizeof(Message::Tcreate) + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint8_t));
    max_len = MAX(max_len, sizeof(Message::Rcreate));
    max_len = MAX(max_len, sizeof(Message::Tread));
    max_len = MAX(max_len, sizeof(Message::Rread));
    max_len = MAX(max_len, sizeof(Message::Twrite));
    max_len = MAX(max_len, sizeof(Message::Rwrite));
    max_len = MAX(max_len, sizeof(Message::Tstat));
    max_len = MAX(max_len, sizeof(Message::Rstat) + 4 * sizeof(uint16_t));
    max_len = MAX(max_len, sizeof(Message::Twstat) + 4 * sizeof(uint16_t));
    max_len = MAX(max_len, sizeof(Message::Twalk) + sizeof(uint16_t));
    max_len = MAX(max_len, sizeof(Message::Rwalk) + sizeof(qid_t));

    if (receive.Rversion.msize < (sizeof(Message::header) + max_len)) {
        return;
    }

    // Get and check string length
    const size_t expected_string_len = receive.header.length - version_len;
    if (receive.Rversion.version_string_len != expected_string_len) {
        return;
    }

    // String should match what was requested
    if (strncmp("9P2000", (char*)&send.buffer[version_len], expected_string_len) != 0) {
        return;
    }

    // Limit to the agreed message length
    bufferLen = receive.Rversion.msize;

    // Try and attach
    request_attach();
}

// Request attach
void NineP2000::request_attach()
{
    state = State::Attach;

    send.header.type = (uint8_t)Type::Tattach;
    send.header.tag = ARRAY_SIZE(request); // Use a tag that will not be used in normal operation
    send.header.length = sizeof(Message::header) + sizeof(Message::Tattach);

    // Use zero file id, no auth
    send.Tattach.fid = 0;
    send.Tattach.afid = 0;

    // User name ArduPilot, no aname.
    if (!add_string(send, "ArduPilot") || !add_string(send, "")) {
        // Negotiated a message length too small for this message!?
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }

    sock->send(send.buffer, send.header.length);
}

// Handle attach response
void NineP2000::handle_attach()
{
    // Fixed length message, header and qid
    if (receive.header.length != (sizeof(Message::header) + sizeof(Message::Rattach))) {
        return;
    }

    // tag should match the request
    if (receive.header.tag != ARRAY_SIZE(request)) {
        return;
    }

    // Expecting a directory
    if (receive.Rattach.qid.type != qidType::QTDIR) {
        return;
    }

    state = State::Mounted;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "9P2000: mounted file system");
}

// Return the next available tag, NOTAG is none free
uint16_t NineP2000::get_free_tag()
{
    WITH_SEMAPHORE(request_sem);

    // Must be mounted for operations to be valid
    if (state != State::Mounted) {
        return NOTAG;
    }

    for (uint8_t i = 0; i < ARRAY_SIZE(request); i++) {
        if (!request[i].active) {
            request[i].active = true;
            return i;
        }
    }

    return NOTAG;
}

// Return true if there is a response for the given tag
bool NineP2000::tag_response(const uint16_t tag)
{
    if (tag >= ARRAY_SIZE(request)) {
        return false;
    }

    WITH_SEMAPHORE(request_sem);
    return request[tag].active && !request[tag].pending;
}

// Return true if there is a response for the given tag with type
bool NineP2000::tag_response_type(const uint16_t tag, const Type type)
{
    WITH_SEMAPHORE(request_sem);
    return tag_response(tag) && (request[tag].expectedType == type);
}

// Called when a command is timed out
void NineP2000::clear_tag(const uint16_t tag)
{
    if (tag >= ARRAY_SIZE(request)) {
        return;
    }

    WITH_SEMAPHORE(request_sem);
    memset(&request[tag], 0, sizeof(request[tag]));
}

// Generate a new unique file id
uint32_t NineP2000::generate_unique_file_id()
{
    // Use array index as file ID, increment by 1 to keep 0 as special case for root.
    WITH_SEMAPHORE(request_sem);

    for (uint8_t i = 0; i < ARRAY_SIZE(fileIds); i++) {
        if (!fileIds[i].active) {
            fileIds[i].active = true;
            fileIds[i].clunked = false;
            return i + 1;
        }
    }
    return 0;
}

// Clear a file id now the file has been closed
void NineP2000::clear_file_id(const uint32_t fileId)
{
    WITH_SEMAPHORE(request_sem);

    const uint32_t index = fileId - 1;

    // Index should be valid and ID should active
    if ((index >= ARRAY_SIZE(fileIds)) || !fileIds[index].active) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }

    fileIds[index].active = false;
}

// Check if a given ID active
bool NineP2000::valid_file_id(const uint32_t fileId)
{
    const uint32_t index = fileId - 1;
    return (index < ARRAY_SIZE(fileIds)) && fileIds[index].active && !fileIds[index].clunked;
}

// Walk to a new file or directory, return tag, NOTAG if failed
uint16_t NineP2000::request_walk(const char* path, const walkType type)
{
    WITH_SEMAPHORE(request_sem);

    // See if there are any tags free
    const uint16_t tag = get_free_tag();
    if (tag == NOTAG) {
        return NOTAG;
    }

    // Get a new file ID
    const uint32_t id = generate_unique_file_id();

    // Check if id is valid
    if (id == 0) {
        clear_tag(tag);
        return NOTAG;
    }

    // Fill in message
    send.header.type = (uint8_t)Type::Twalk;
    send.header.tag = tag;
    send.header.length = sizeof(Message::header) + sizeof(Message::Twalk);

    // Start at root
    send.Twalk.fid = 0;

    // End at new id
    send.Twalk.newfid = id;

    // Zero steps to start with
    send.Twalk.nwname = 0;

    // Step through path and add strings
    const uint16_t len = strlen(path);
    uint16_t name_start = 0; 
    for (uint16_t i = 0; i < len; i++) {
        const bool split = path[i] == '/';
        const bool last = i == (len - 1);
        if (split || last) {
            // Should check total length of message is still valid
            const uint16_t name_len = i - name_start + (split ? 0 : 1);
            const uint32_t new_len = send.header.length + sizeof(name_len) + name_len;

            // Check total message length is still valid
            if (new_len > bufferLen) {
                clear_tag(tag);
                return NOTAG;
            }

            // Add string length
            memcpy(&send.buffer[send.header.length], &name_len, sizeof(name_len));
            send.header.length += sizeof(name_len);

            // Add string itself
            memcpy(&send.buffer[send.header.length], &path[name_start], name_len);
            send.header.length += name_len;

            // Increment number of names
            send.Twalk.nwname += 1;

            // Next section starts after this one
            name_start = i + 1;
        }
    }

    // Make tag as active
    request[tag].pending = true;
    request[tag].expectedType = Type::Rwalk;
    request[tag].walk.fileId = id;
    request[tag].walk.type = type;

    // Send command
    sock->send(send.buffer, send.header.length);

    return tag;
}

void NineP2000::handle_error(Request& result)
{
    // null terminate string and print directly out of buffer
    const uint16_t len = receive.Rerror.ename_string_len;
    const uint32_t string_start = sizeof(Message::header) + sizeof(Message::Rerror);
    receive.buffer[string_start + MIN(len, 100)] = 0;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "9P2000: error: %s", (char*)&receive.buffer[string_start]);

    // Set the appropiate fail response for the expected message
    switch (result.expectedType) {
        case Type::Rwalk:
            result.walk.fileId = 0;
            break;

        case Type::Ropen:
            result.open.result = false;
            break;

        case Type::Rcreate:
            result.create.result = false;
            break;

        case Type::Rread:
            result.read.count = -1;
            break;

        case Type::Rwrite:
            result.write.count = -1;
            break;

        case Type::Rremove:
            result.remove.result = false;
            break;

        case Type::Rstat:
            result.stat.result = false;
            break;

        case Type::Rwstat:
            result.rwstat.result = false;
            break;

        default:
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
    }
}

void NineP2000::handle_rwalk(Request& result)
{
    WITH_SEMAPHORE(request_sem);

    const uint16_t num_ids = receive.Rwalk.nwqid;

    // Zero length walk means we must be root
    if (num_ids == 0) {
        // Root must be a dir
        if (result.walk.type != walkType::Directory) {
            free_file_id(result.walk.fileId);
            result.walk.fileId = 0;
            return;
        }
    }

    // Calculate the offset of the last id
    const uint16_t id_offset = sizeof(Message::header) + sizeof(Message::Rwalk) + (num_ids - 1) * sizeof(qid_t);

    // Make sure the message is long enough
    if (receive.header.length != (id_offset + sizeof(qid_t))) {
        free_file_id(result.walk.fileId);
        result.walk.fileId = 0;
        return;
    }

    // Expecting the correct type
    if (result.walk.type != walkType::Any) {

        // Read in last id
        const qid_t &qid = (qid_t&)receive.buffer[id_offset];

        const uint8_t expected_type = (result.walk.type == walkType::Directory) ? qidType::QTDIR : qidType::QTFILE;
        if (qid.type != expected_type) {
            free_file_id(result.walk.fileId);
            result.walk.fileId = 0;
            return;
        }
    }

    // Got this far then fileId is valid, wait for the caller to pickup the result
}

uint32_t NineP2000::walk_result(const uint16_t tag)
{
    WITH_SEMAPHORE(request_sem);

    // Make sure the tag is valid and there is a waiting response
    if (!tag_response_type(tag, Type::Rwalk)) {
        clear_tag(tag);
        return 0;
    }

    // Get file id
    const uint32_t fileId = request[tag].walk.fileId;

    // success, return id
    clear_tag(tag);
    return fileId;
}

// Return the file id to the server for re-use
void NineP2000::free_file_id(const uint32_t id)
{
    WITH_SEMAPHORE(request_sem);

    // Check id is valid
    if (!valid_file_id(id)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    // See if there are any tags free
    const uint16_t tag = get_free_tag();
    if (tag == NOTAG) {
        // This is bad, it means we leak a file id.
        return;
    }

    // Mark as clunked so we only free once
    fileIds[id - 1].clunked = true;

    // Mark tag as active
    request[tag].pending = true;
    request[tag].expectedType = Type::Rclunk;
    request[tag].clunk.fileId = id;

    // Fill in message
    send.header.type = (uint8_t)Type::Tclunk;
    send.header.tag = tag;
    send.header.length = sizeof(Message::header) + sizeof(Message::Tclunk);
    send.Tclunk.fid = id;

    sock->send(send.buffer, send.header.length);
}

// Request read of given file or directory with given flags
uint16_t NineP2000::request_open(const uint32_t id, const int flags)
{
    WITH_SEMAPHORE(request_sem);

    // See if there are any tags free
    const uint16_t tag = get_free_tag();
    if (tag == NOTAG) {
        return NOTAG;
    }

    // ID invalid
    if (!valid_file_id(id)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return NOTAG;
    }

    // Mark tag as active
    request[tag].pending = true;
    request[tag].expectedType = Type::Ropen;

    // Translate flags to mode
    uint8_t mode = 0;
    if ((flags & O_RDWR) != 0) {
        mode = openMode::ORDWR;

    } else if ((flags & O_WRONLY) != 0) {
        mode = openMode::OWRITE;

    } else {
        mode = openMode::OREAD;
    }

    // Fill in message
    send.header.type = (uint8_t)Type::Topen;
    send.header.tag = tag;
    send.header.length = sizeof(Message::header) + sizeof(Message::Topen);
    send.Topen.fid = id;
    send.Topen.mode = mode;

    sock->send(send.buffer, send.header.length);

    return tag;
}

// Get open result, returns true if successful
bool NineP2000::open_result(const uint16_t tag)
{
    WITH_SEMAPHORE(request_sem);

    // Make sure the tag is valid and there is a waiting response
    if (!tag_response_type(tag, Type::Ropen)) {
        clear_tag(tag);
        return false;
    }

    const bool ret = request[tag].open.result;

    clear_tag(tag);
    return ret;
}

// Return the maximum length that can be read in a single packet
// This is only valid if the file system is mounted as bufferlength is negotiated
uint32_t NineP2000::max_read_len() const {
    const uint16_t data_offset = sizeof(Message::header) + sizeof(Message::Rread);
    return bufferLen - data_offset;
}

// Read a directory, return tag, NOTAG if failed
uint16_t NineP2000::request_dir_read(const uint32_t id, const uint64_t offset, struct dirent *de)
{
    WITH_SEMAPHORE(request_sem);

    // ID invalid
    if (!valid_file_id(id)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return NOTAG;
    }

    // See if there are any tags free
    const uint16_t tag = get_free_tag();
    if (tag == NOTAG) {
        return NOTAG;
    }

    // Mark tag as active
    request[tag].pending = true;
    request[tag].expectedType = Type::Rread;
    request[tag].read.is_dir = true;
    request[tag].read.dir = de;

    // Fill in message
    send.header.type = (uint8_t)Type::Tread;
    send.header.tag = tag;
    send.header.length = sizeof(Message::header) + sizeof(Message::Tread);
    send.Tread.fid = id;
    send.Tread.offset = offset;

    // We don't know how long the directory entry will be as it has variable length strings
    // Just read max length for now
    send.Tread.count = max_read_len();

    sock->send(send.buffer, send.header.length);

    return tag;
}

void NineP2000::handle_dir_Rread(Request& result)
{
    WITH_SEMAPHORE(request_sem);

    // Need a place to put the result
    if (result.read.dir == nullptr) {
        return;
    }

    // Should at least contain header count and stat
    const uint16_t stat_end = sizeof(Message::header) + sizeof(Message::Rread) + sizeof(stat_t);
    if (receive.header.length < stat_end) {
        return;
    }

    const stat_t &info = (stat_t&)receive.buffer[sizeof(Message::header) + sizeof(Message::Rread)];

    // Length feild does not include itself
    const uint16_t stat_len = info.msg_size + sizeof(stat_t::msg_size);

    // Make sure there is room for the whole stat now we know the full size
    if (receive.header.length < (sizeof(Message::Rread) + stat_len)) {
        return;
    }

    // Check file mode
    if ((info.qid.type != qidType::QTFILE) && (info.qid.type != qidType::QTDIR)) {
        return;
    }

    // All checks done, now we can update the directory entry

    // Copy name, comes after the stat
    uint16_t name_len = 0;
    memcpy(&name_len, &receive.buffer[stat_end], sizeof(name_len));

    memset(result.read.dir->d_name, 0, sizeof(result.read.dir->d_name));
    strncpy(result.read.dir->d_name, (char*)&receive.buffer[stat_end + sizeof(name_len)], MIN(sizeof(result.read.dir->d_name), name_len));

    // Fill in file flags
    if (info.qid.type == qidType::QTFILE) {
        result.read.dir->d_type = DT_REG;
    } else{
        result.read.dir->d_type = DT_DIR;
    }

    result.read.count = stat_len;
}

// Read a file, return tag, NOTAG if failed
uint16_t NineP2000::request_file_read(const uint32_t id, const uint64_t offset, const uint32_t count, void *buf)
{
    WITH_SEMAPHORE(request_sem);

    // ID invalid
    if (!valid_file_id(id)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return NOTAG;
    }

    // See if there are any tags free
    const uint16_t tag = get_free_tag();
    if (tag == NOTAG) {
        return NOTAG;
    }

    // Mark tag as active
    request[tag].pending = true;
    request[tag].expectedType = Type::Rread;
    request[tag].read.is_dir = false;
    request[tag].read.buf = buf;
    request[tag].read.count = count;

    // Fill in message
    send.header.type = (uint8_t)Type::Tread;
    send.header.tag = tag;
    send.header.length = sizeof(Message::header) + sizeof(Message::Tread);
    send.Tread.fid = id;
    send.Tread.offset = offset;
    send.Tread.count = count;

    sock->send(send.buffer, send.header.length);

    return tag;
}

void NineP2000::handle_file_Rread(Request& result)
{
    WITH_SEMAPHORE(request_sem);

    // Need a place to put the result
    if (result.read.buf == nullptr) {
        result.read.count = -1;
        return;
    }

    // Not expecting to get more data than was asked for
    if (receive.Rread.count > uint32_t(result.read.count)) {
        result.read.count = -1;
        return;
    }
    result.read.count = receive.Rread.count;

    // Copy result
    memcpy(result.read.buf, &receive.buffer[sizeof(Message::header) + sizeof(Message::Rread)], result.read.count);
}

// Fill in a directory item based on the read result, returns none zero if success
int32_t NineP2000::read_result(const uint16_t tag, const bool is_dir)
{
    WITH_SEMAPHORE(request_sem);

    // Make sure the tag is valid and there is a waiting response
    if (!tag_response_type(tag, Type::Rread)) {
        clear_tag(tag);
        return -1;
    }

    // Should be for the expected type
    if (request[tag].read.is_dir != is_dir) {
        clear_tag(tag);
        return -1;
    }

    const int32_t count = request[tag].read.count;

    clear_tag(tag);
    return count;
}

// Request stat for a given file id
uint16_t NineP2000::request_stat(const uint32_t id, struct stat *stbuf)
{
    WITH_SEMAPHORE(request_sem);

    // ID invalid
    if (!valid_file_id(id)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return NOTAG;
    }

    // See if there are any tags free
    const uint16_t tag = get_free_tag();
    if (tag == NOTAG) {
        return NOTAG;
    }

    // Mark tag as active
    request[tag].pending = true;
    request[tag].expectedType = Type::Rstat;
    request[tag].stat.stbuf = stbuf;

    // Fill in message
    send.header.type = (uint8_t)Type::Tstat;
    send.header.tag = tag;
    send.header.length = sizeof(Message::header) + sizeof(Message::Tstat);
    send.Tstat.fid = id;

    sock->send(send.buffer, send.header.length);

    return tag;
}

// Handle a stat response
void NineP2000::handle_Rstat(Request& result)
{
    WITH_SEMAPHORE(request_sem);

    // Clear stats
    memset(result.stat.stbuf, 0, sizeof(*result.stat.stbuf));

    // length in bytes
    result.stat.stbuf->st_size = receive.Rstat.stat.length;

    // Access and modification timestamps
    result.stat.stbuf->st_atime = receive.Rstat.stat.atime;
    result.stat.stbuf->st_mtime = receive.Rstat.stat.mtime;

    // Fill in file flags
    const uint8_t &mode = receive.Rstat.stat.qid.type;
    if (mode == qidType::QTFILE) {
        result.stat.stbuf->st_mode |= S_IFREG;
    } else if (mode == qidType::QTDIR) {
        result.stat.stbuf->st_mode |= S_IFDIR;
    }

    result.stat.result = true;
}

// Get stat result, returns true if successful
bool NineP2000::stat_result(const uint16_t tag)
{
    WITH_SEMAPHORE(request_sem);

    // Make sure the tag is valid and there is a waiting response
    if (!tag_response_type(tag, Type::Rstat)) {
        clear_tag(tag);
        return false;
    }

    const bool ret = request[tag].stat.result;

    clear_tag(tag);
    return ret;
}

// Return the maximum length that can be written in a single packet
// This is only valid if the file system is mounted as bufferlength is negotiated
uint32_t NineP2000::max_write_len() const {
    const uint16_t data_offset = sizeof(Message::header) + sizeof(Message::Twrite);
    return bufferLen - data_offset;
}

// Request write for given file id, return tag
uint16_t NineP2000::request_write(const uint32_t id, const uint64_t offset, uint32_t count, const void *buf)
{
    WITH_SEMAPHORE(request_sem);

    // ID invalid
    if (!valid_file_id(id)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return NOTAG;
    }

    // See if there are any tags free
    const uint16_t tag = get_free_tag();
    if (tag == NOTAG) {
        return NOTAG;
    }

    // Limit write to max packet size
    const uint16_t data_offset = sizeof(Message::header) + sizeof(Message::Twrite);
    count = MIN(count, uint32_t(bufferLen - data_offset));

    // Mark tag as active
    request[tag].pending = true;
    request[tag].expectedType = Type::Rwrite;

    // Fill in message
    send.header.type = (uint8_t)Type::Twrite;
    send.header.tag = tag;
    send.header.length = data_offset + count;
    send.Twrite.fid = id;
    send.Twrite.offset = offset;
    send.Twrite.count = count;

    memcpy(&send.buffer[data_offset], buf, count);

    sock->send(send.buffer, send.header.length);

    return tag;
}

int32_t NineP2000::write_result(const uint16_t tag)
{
    WITH_SEMAPHORE(request_sem);

    // Make sure the tag is valid and there is a waiting response
    if (!tag_response_type(tag, Type::Rwrite)) {
        clear_tag(tag);
        return -1;
    }

    // Should be a create response
    if (request[tag].expectedType != Type::Rwrite) {
        clear_tag(tag);
        return -1;
    }

    const int32_t ret = request[tag].write.count;

    clear_tag(tag);

    return ret;
}

// Request create for given directory id, return tag
uint16_t NineP2000::request_create(const uint32_t id, const char*name, const bool dir)
{
    WITH_SEMAPHORE(request_sem);

    // ID invalid
    if (!valid_file_id(id)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return NOTAG;
    }

    // See if there are any tags free
    const uint16_t tag = get_free_tag();
    if (tag == NOTAG) {
        return NOTAG;
    }

    // Mark tag as active
    request[tag].pending = true;
    request[tag].expectedType = Type::Rcreate;

    // Fill in message
    send.header.type = (uint8_t)Type::Tcreate;
    send.header.tag = tag;
    send.header.length = sizeof(Message::header) + sizeof(Message::Tcreate);
    send.Tcreate.fid = id;

    // Give everyone rwx permissions
    const uint32_t perm = 0777 | (dir ? (qidType::QTDIR << 24) : 0);
    const uint8_t mode = 0;

    // permissions and mode come after the variable length string
    const uint8_t tail_len = sizeof(perm) + sizeof(mode);

    if (!add_string(send, name) || ((send.header.length + tail_len) > bufferLen)) {
        // Ran out of room in the message
        clear_tag(tag);
        return NOTAG;
    }

    // Fill in tail now string length is set
    memcpy(&send.buffer[send.header.length], &perm, sizeof(perm));
    send.header.length += sizeof(perm);
    memcpy(&send.buffer[send.header.length], &mode, sizeof(mode));
    send.header.length += sizeof(mode);


    sock->send(send.buffer, send.header.length);

    return tag;
}

bool NineP2000::create_result(const uint16_t tag)
{
    WITH_SEMAPHORE(request_sem);

    // Make sure the tag is valid and there is a waiting response
    if (!tag_response_type(tag, Type::Rcreate)) {
        clear_tag(tag);
        return false;
    }

    const bool ret = request[tag].create.result;

    clear_tag(tag);

    return ret;
}

// Request remove for given id, return tag
uint16_t NineP2000::request_remove(const uint32_t id)
{
    WITH_SEMAPHORE(request_sem);

    // ID invalid
    if (!valid_file_id(id)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return NOTAG;
    }

    // See if there are any tags free
    const uint16_t tag = get_free_tag();
    if (tag == NOTAG) {
        return NOTAG;
    }

    // Mark tag as active
    request[tag].pending = true;
    request[tag].expectedType = Type::Rremove;

    // Fill in message
    send.header.type = (uint8_t)Type::Tremove;
    send.header.tag = tag;
    send.header.length = sizeof(Message::header) + sizeof(Message::Tremove);

    send.Tremove.fid = id;

    sock->send(send.buffer, send.header.length);

    return tag;
}

// Get result of remove
bool NineP2000::remove_result(const uint16_t tag)
{
    WITH_SEMAPHORE(request_sem);

    // Make sure the tag is valid and there is a waiting response
    if (!tag_response_type(tag, Type::Rremove)) {
        clear_tag(tag);
        return false;
    }

    const bool ret = request[tag].remove.result;

    clear_tag(tag);

    return ret;
}

// Request rename for given id, return tag
uint16_t NineP2000::request_rename(const uint32_t id, const char*name)
{
    WITH_SEMAPHORE(request_sem);

    // ID invalid
    if (!valid_file_id(id)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return NOTAG;
    }

    // See if there are any tags free
    const uint16_t tag = get_free_tag();
    if (tag == NOTAG) {
        return NOTAG;
    }

    // Mark tag as active
    request[tag].pending = true;
    request[tag].expectedType = Type::Rwstat;

    // Fill in message
    send.header.type = (uint8_t)Type::Twstat;
    send.header.tag = tag;
    send.header.length = sizeof(Message::header) + sizeof(Message::Twstat);

    send.Twstat.fid = id;
    send.Twstat.nstat = 1;

    // Max values indicate don't change.
    memset(&send.Twstat.stat, 0xFF, sizeof(send.Twstat.stat));

    // Add new name
    if (!add_string(send, name) ||
        // Don't change other names
        !add_string(send, "") ||
        !add_string(send, "") ||
        !add_string(send, "")) {

        // Ran out of room in the message
        clear_tag(tag);
        return NOTAG;
    }

    sock->send(send.buffer, send.header.length);

    return tag;
}

// Get result of rename and mtime set
bool NineP2000::stat_update_result(const uint16_t tag)
{
    WITH_SEMAPHORE(request_sem);

    // Make sure the tag is valid and there is a waiting response
    if (!tag_response_type(tag, Type::Rwstat)) {
        clear_tag(tag);
        return false;
    }

    const bool ret = request[tag].rwstat.result;

    // Finished with tag
    clear_tag(tag);

    return ret;
}

// Request mtime update for given id, return tag
uint16_t NineP2000::request_set_mtime(const uint32_t id, const uint32_t mtime)
{
    WITH_SEMAPHORE(request_sem);

    // ID invalid
    if (!valid_file_id(id)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return NOTAG;
    }

    // See if there are any tags free
    const uint16_t tag = get_free_tag();
    if (tag == NOTAG) {
        return NOTAG;
    }

    // Mark tag as active
    request[tag].pending = true;
    request[tag].expectedType = Type::Rwstat;

    // Fill in message
    send.header.type = (uint8_t)Type::Twstat;
    send.header.tag = tag;
    send.header.length = sizeof(Message::header) + sizeof(Message::Twstat);

    send.Twstat.fid = id;
    send.Twstat.nstat = 1;

    // Max values indicate don't change.
    memset(&send.Twstat.stat, 0xFF, sizeof(Message::Twstat.stat));

    // Set mtime
    send.Twstat.stat.mtime = mtime;

    // Don't change names
    if (!add_string(send, "") ||
        !add_string(send, "") ||
        !add_string(send, "") ||
        !add_string(send, "")) {

        // Ran out of room in the message
        clear_tag(tag);
        return NOTAG;
    }

    sock->send(send.buffer, send.header.length);

    return tag;
}

#endif // AP_NETWORKING_FILESYSTEM_ENABLED
