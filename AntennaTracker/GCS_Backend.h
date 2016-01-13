#include "GCS_MAVLink/GCS.h"

class GCS_Backend_Tracker : public GCS_MAVLINK {

public:

    bool stream_trigger(enum streams stream_num) override;
    void data_stream_send(void) override;
    void handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override;
    void handleMessage(mavlink_message_t* msg) override;

protected:

    AP_GPS &_gps() const override;

#if AP_AHRS_NAVEKF_AVAILABLE
    AP_AHRS_NavEKF &_ahrs() const override;
#else
    AP_AHRS &_ahrs() const override;
#endif

    Compass &_compass() const override;

    bool should_try_send_message(enum ap_message id) override;

    bool send_ATTITUDE() override;
    bool send_GLOBAL_POSITION_INT() override;
    bool send_HEARTBEAT() override;
    bool send_NAV_CONTROLLER_OUTPUT() override;
    bool send_RAW_IMU() override;
    bool send_RC_CHANNELS_RAW() override;
    bool send_SCALED_PRESSURE() override;
    bool send_SENSOR_OFFSETS() override;
    bool send_SERVO_OUTPUT_RAW() override;
    bool send_SIMSTATE() override;
    bool send_STATUSTEXT() override;

private:

};
