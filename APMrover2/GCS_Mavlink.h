#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_Rover : public GCS_MAVLINK
{
public:

protected:

    uint32_t telem_delay() const override;
    bool accept_packet(const mavlink_status_t &status, mavlink_message_t &msg) override;

    Compass *get_compass() const override;
    AP_Mission *get_mission() override;
    AP_Rally *get_rally() const override { return nullptr; };
    AP_Camera *get_camera() const override;
    AP_AdvancedFailsafe *get_advanced_failsafe() const override;

    uint8_t sysid_my_gcs() const override;

    bool set_mode(uint8_t mode) override;

    MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_long_t &packet) override;
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet) override;
    MAV_RESULT handle_command_long_packet(const mavlink_command_long_t &packet) override;

    virtual bool in_hil_mode() const override;

    bool persist_streamrates() const override { return true; }

private:

    void handleMessage(mavlink_message_t * msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override;
    bool try_send_message(enum ap_message id) override;

    MAV_TYPE frame_type() const override;
    MAV_MODE base_mode() const override;
    uint32_t custom_mode() const override;
    MAV_STATE system_status() const override;

    int16_t vfr_hud_throttle() const override;

};
