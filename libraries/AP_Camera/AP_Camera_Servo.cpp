#include "AP_Camera_Servo.h"

#if AP_CAMERA_SERVO_ENABLED

#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;


void AP_Camera_Servo::init() {
    SRV_Channels::set_output_scaled(SRV_Channel::k_cam_zoom, 500);
    SRV_Channels::set_output_scaled(SRV_Channel::k_cam_focus, 500);
}

// update - should be called at 50hz
void AP_Camera_Servo::update()
{
    // shutter counter
    if (trigger_counter > 0) {
        trigger_counter--;
    } else {
        SRV_Channels::set_output_pwm(SRV_Channel::k_cam_trigger, _params.servo_off_pwm);
    }

    // iso counter
    if (iso_counter > 0) {
        iso_counter--;
    } else {
        SRV_Channels::set_output_pwm(SRV_Channel::k_cam_iso, _params.servo_off_pwm);
    }

    // call parent update
    AP_Camera_Backend::update();
}

// entry point to actually take a picture.  returns true on success
bool AP_Camera_Servo::trigger_pic()
{
    // fail if have not completed previous picture
    if (trigger_counter > 0) {
        return false;
    }

    SRV_Channels::set_output_pwm(SRV_Channel::k_cam_trigger, _params.servo_on_pwm);

    // set counter to move servo to off position after this many iterations of update (assumes 50hz update rate)
    trigger_counter = constrain_float(_params.trigger_duration * 50, 0, UINT16_MAX);

    return true;
}


bool AP_Camera_Servo::set_zoom(ZoomType zoom_type, float zoom_value) {
    if (zoom_type == ZoomType::RATE) {
        // set zoom rate
        float current_zoom = SRV_Channels::get_output_scaled(SRV_Channel::k_cam_zoom);
        float new_zoom = current_zoom + zoom_value * 10;
        SRV_Channels::set_output_scaled(SRV_Channel::k_cam_zoom, new_zoom);
        return true;
    }
    return false;
}

// set focus specified as rate, percentage or auto
// focus in = -1, focus hold = 0, focus out = 1
SetFocusResult AP_Camera_Servo::set_focus(FocusType focus_type, float focus_value) {
    if (focus_type == FocusType::RATE) {    
        // set zoom rate
        const float current_focus = SRV_Channels::get_output_scaled(SRV_Channel::k_cam_focus);
        const float new_focus = current_focus + focus_value * 10;
        SRV_Channels::set_output_scaled(SRV_Channel::k_cam_focus, new_focus);
        return SetFocusResult::ACCEPTED;
    }
    return SetFocusResult::UNSUPPORTED;
}

// configure camera
void AP_Camera_Servo::configure(float shooting_mode, float shutter_speed, float aperture, float ISO, int32_t exposure_type, int32_t cmd_id, float engine_cutoff_time)
{
    // designed to control Blackmagic Micro Cinema Camera (BMMCC) cameras
    // if the message contains non zero values then use them for the below functions
    if (ISO > 0) {
        // set a trigger for the iso function that is flip controlled
        iso_counter = constrain_float(_params.trigger_duration * 50, 0, UINT16_MAX);
        SRV_Channels::set_output_pwm(SRV_Channel::k_cam_iso, _params.servo_on_pwm);
    }

    if (aperture > 0) {
        SRV_Channels::set_output_pwm(SRV_Channel::k_cam_aperture, (uint16_t)aperture);
    }

    if (shutter_speed > 0) {
        SRV_Channels::set_output_pwm(SRV_Channel::k_cam_shutter_speed, (uint16_t)shutter_speed);
    }

    // Use the shooting mode PWM value for the BMMCC as the focus control - no need to modify or create a new MAVlink message type.
    if (shooting_mode > 0) {
        SRV_Channels::set_output_pwm(SRV_Channel::k_cam_focus, (uint16_t)shooting_mode);
    }
}

#endif // AP_CAMERA_SERVO_ENABLED
