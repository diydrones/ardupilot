#pragma once

/// @file    AP_L1_Control.h
/// @brief   L1 Control algorithm. This is a instance of an
/// AP_Navigation class

/*
 * Originally written by Brandon Jones 2013
 *
 *  Modified by Paul Riseborough 2013 to provide:
 *  - Explicit control over frequency and damping
 *  - Explicit control over track capture angle
 *  - Ability to use loiter radius smaller than L1 length
 */

#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Navigation/AP_Navigation.h>
#include <AP_TECS/AP_TECS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Vehicle/AP_FixedWing.h>
#include <AP_Common/Location.h>

class AP_L1_Control : public AP_Navigation {
public:
    AP_L1_Control(const AP_AHRS &ahrs, const AP_TECS &tecs, const AP_FixedWing &aparm)
        : _ahrs(ahrs)
        , _tecs(tecs)
        , _baro(*AP_Baro::get_singleton())
        , _aparm(aparm)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_L1_Control);

    /* see AP_Navigation.h for the definitions and units of these
     * functions */
    int32_t nav_roll_cd(void) const override;
    float lateral_acceleration(void) const override;

    // return the desired track heading angle(centi-degrees)
    int32_t nav_bearing_cd(void) const override;

    // return the heading error angle (centi-degrees) +ve to left of track
    int32_t bearing_error_cd(void) const override;

    float crosstrack_error(void) const override { return _crosstrack_error; }
    float crosstrack_error_integrator(void) const override { return _L1_xtrack_i; }

    int32_t target_bearing_cd(void) const override;
    float turn_distance(float wp_radius) const override;
    float turn_distance(float wp_radius, float turn_angle) const override;
    void update_waypoint(const class Location &prev_WP, const class Location &next_WP, float dist_min = 0.0f) override;
    bool is_loiter_achievable(float radius, float indicated_airspeed,
                              float altitude_amsl) const override;
    float calc_corrected_loiter_radius(float original_radius,
                                       float indicated_airspeed,
                                       float altitude_amsl) const override;
    void update_loiter(const class Location &center_WP, float radius, int8_t loiter_direction) override;
    void update_heading_hold(int32_t navigation_heading_cd) override;
    void update_level_flight(void) override;
    bool reached_loiter_target(void) override;

    // set the default NAVL1_PERIOD
    void set_default_period(float period) {
        _L1_period.set_default(period);
    }

    void set_data_is_stale(void) override {
        _data_is_stale = true;
    }
    bool data_is_stale(void) const override {
        return _data_is_stale;
    }

    // this supports the NAVl1_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    void set_reverse(bool reverse) override {
        _reverse = reverse;
    }

private:
    // reference to the AHRS object
    const AP_AHRS &_ahrs;

    // reference to the TECS object
    const AP_TECS &_tecs;

    // reference to the barometer object
    const AP_Baro &_baro;

    // reference to the fixed wing parameters object
    const AP_FixedWing &_aparm;

    enum class NavMode {
        NONE,
        WAYPOINT,
        LOITER,
        HEADING_HOLD,
        LEVEL_FLIGHT
    };
    NavMode _current_nav_mode = NavMode::NONE;

    // lateral acceration in m/s required to fly to the
    // L1 reference point (+ve to right)
    float _latAccDem;

    // L1 tracking distance in meters which is dynamically updated
    float _L1_dist;

    // Status which is true when the vehicle has started circling the WP
    bool _WPcircle;

    // bearing angle (radians) to L1 point
    float _nav_bearing;

    // bearing error angle (radians) +ve to left of track
    float _bearing_error;

    // crosstrack error in meters
    float _crosstrack_error;

    // target bearing in centi-degrees from last update
    int32_t _target_bearing_cd;

    // L1 tracking loop period (sec)
    AP_Float _L1_period;
    // L1 tracking loop damping ratio
    AP_Float _L1_damping;

    // previous value of cross-track velocity
    float _last_Nu;

    // prevent indecision in waypoint tracking
    void _prevent_indecision(float &Nu);

    // integral feedback to correct crosstrack error. Used to ensure xtrack converges to zero.
    // For tuning purposes it's helpful to clear the integrator when it changes so a _prev is used
    void _update_xtrack_integral(float error, float max_abs_error, float clamp);
    uint32_t _last_update_xtrack_i_us;
    float _L1_xtrack_i = 0;
    AP_Float _L1_xtrack_i_gain;
    float _L1_xtrack_i_gain_prev = 0;
    bool _data_is_stale = true;

    // calculate minimum achievable turn radius based on indicated airspeed and
    // altitude AMSL (assuming steady, coordinated, level turn)
    float _calc_min_turn_radius(float ias = NAN, float altitude_amsl = NAN) const;

    // remember reached_loiter_target decision
    struct {
        uint32_t reached_loiter_target_ms;
        float radius;
        int8_t direction;
        Location center_WP;
    } _last_loiter;

    bool _reverse = false;
    float get_yaw() const;
    int32_t get_yaw_sensor() const;
};
