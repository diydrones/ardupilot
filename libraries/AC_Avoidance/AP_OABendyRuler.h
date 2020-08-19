#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

/*
 * BendyRuler avoidance algorithm for avoiding the polygon and circular fence and dynamic objects detected by the proximity sensor
 */
class AP_OABendyRuler {
public:
    AP_OABendyRuler();

    /* Do not allow copies */
    AP_OABendyRuler(const AP_OABendyRuler &other) = delete;
    AP_OABendyRuler &operator=(const AP_OABendyRuler&) = delete;

    // send configuration info stored in front end parameters
    void set_config(float margin_max) { _margin_max = MAX(margin_max, 0.0f); }

    // run background task to find best path and update avoidance_results
    // returns true and populates origin_new and destination_new if OA is required.  returns false if OA is not required
    bool update(const Location& current_loc, const Location& destination, const Vector2f &ground_speed_vec, Location &origin_new, Location &destination_new);

    static const struct AP_Param::GroupInfo var_info[];

private:

    // calculate minimum distance between a path and any obstacle
    float calc_avoidance_margin(const Location &start, const Location &end) const;

    // determine if BendyRuler should accept the new bearing or try and resist it. Returns true if bearing is not changed  
    bool resist_bearing_change(const Location &destination, const Location &current_loc, bool active, float bearing_test, float lookahead_step1_dist, float margin, Location &prev_dest, float &prev_bearing, float &final_bearing, float &final_margin) const;    

    // calculate minimum distance between a path and the circular fence (centered on home)
    // on success returns true and updates margin
    bool calc_margin_from_circular_fence(const Location &start, const Location &end, float &margin) const;

    // calculate minimum distance between a path and all inclusion and exclusion polygons
    // on success returns true and updates margin
    bool calc_margin_from_inclusion_and_exclusion_polygons(const Location &start, const Location &end, float &margin) const;

    // calculate minimum distance between a path and all inclusion and exclusion circles
    // on success returns true and updates margin
    bool calc_margin_from_inclusion_and_exclusion_circles(const Location &start, const Location &end, float &margin) const;

    // calculate minimum distance between a path and proximity sensor obstacles
    // on success returns true and updates margin
    bool calc_margin_from_object_database(const Location &start, const Location &end, float &margin) const;

    // OA common parameters
    float _margin_max;              // object avoidance will ignore objects more than this many meters from vehicle
    
    // BendyRuler parameters
    AP_Float _lookahead;            // object avoidance will look this many meters ahead of vehicle
    AP_Float _bendy_ratio;          // object avoidance will avoid major directional change if change in margin ratio is less than this param
    AP_Int16 _bendy_angle;          // object avoidance will try avoding change in direction over this much angle

    // internal variables used by background thread
    float _current_lookahead;       // distance (in meters) ahead of the vehicle we are looking for obstacles
    float _bearing_prev;            // stored bearing in degrees 
    Location _destination_prev;     // previous destination, to check if there has been a change in destination
};
