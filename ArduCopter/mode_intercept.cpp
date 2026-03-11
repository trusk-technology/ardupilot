#include "Copter.h"

#if MODE_INTERCEPT_ENABLED

#include <AP_Seeker/AP_Seeker.h>

/*
 * mode_intercept.cpp — FOV-holding seeker intercept for ArduCopter
 *
 * Controller design (arxiv 2409.17497):
 *   - Horizontal tracking: null centroid_x via yaw rate (avoids pitch-tilt seeker coupling)
 *   - Vertical tracking:   drive centroid_y to zero via vertical velocity command
 *   - Acceleration comp:   forward body accel → upward velocity to cancel pitch-tilt disturbance
 *   - Forward speed:       constant INTC_SPEED in current heading direction
 */

const AP_Param::GroupInfo ModeIntercept::var_info[] = {
    // @Param: SPEED
    // @DisplayName: Intercept forward speed
    // @Description: Forward approach speed in INTERCEPT mode.
    // @Range: 0 20
    // @Units: m/s
    AP_GROUPINFO("SPEED",  0, ModeIntercept, speed,      3.0f),

    // @Param: YAW_P
    // @DisplayName: Intercept yaw proportional gain
    // @Description: Yaw rate proportional gain. Converts centroid_x (fraction of FOV) to yaw rate (rad/s).
    // @Range: 0 10
    AP_GROUPINFO("YAW_P",  1, ModeIntercept, yaw_p,      2.0f),

    // @Param: YAW_D
    // @DisplayName: Intercept yaw derivative gain
    // @Description: Yaw rate derivative gain. Converts los_rate_x (rad/s) to yaw rate (rad/s).
    // @Range: 0 2
    AP_GROUPINFO("YAW_D",  2, ModeIntercept, yaw_d,      0.3f),

    // @Param: VRT_P
    // @DisplayName: Intercept vertical velocity gain
    // @Description: Vertical velocity gain. Converts centroid_y (fraction of FOV) to vertical speed (m/s).
    // @Range: 0 10
    // @Units: m/s
    AP_GROUPINFO("VRT_P",  3, ModeIntercept, vrt_p,      3.0f),

    // @Param: ACMP
    // @DisplayName: Intercept acceleration compensation gain
    // @Description: Converts forward body acceleration (m/s^2) to upward velocity (m/s) to cancel pitch-tilt seeker disturbance.
    // @Range: 0 2
    AP_GROUPINFO("ACMP",   4, ModeIntercept, accel_comp, 0.5f),

    // @Param: TOUT
    // @DisplayName: Intercept seeker timeout
    // @Description: Time in milliseconds after which missing SEEKER_TARGET data triggers a position hold.
    // @Range: 100 5000
    // @Units: ms
    AP_GROUPINFO("TOUT",   5, ModeIntercept, timeout_ms, 500.0f),

    AP_GROUPEND
};

bool ModeIntercept::init(bool ignore_checks)
{
    if (!copter.position_ok()) {
        return false;
    }

    // initialise horizontal speed and acceleration limits
    pos_control->NE_set_max_speed_accel_m(wp_nav->get_default_speed_NE_ms(), wp_nav->get_wp_acceleration_mss());
    pos_control->NE_set_correction_speed_accel_m(wp_nav->get_default_speed_NE_ms(), wp_nav->get_wp_acceleration_mss());

    // initialise vertical speed and acceleration limits
    pos_control->D_set_max_speed_accel_m(wp_nav->get_default_speed_down_ms(), wp_nav->get_default_speed_up_ms(), wp_nav->get_accel_D_mss());
    pos_control->D_set_correction_speed_accel_m(wp_nav->get_default_speed_down_ms(), wp_nav->get_default_speed_up_ms(), wp_nav->get_accel_D_mss());

    // initialise position controllers
    pos_control->NE_init_controller();
    pos_control->D_init_controller();

    // hold current yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    return true;
}

void ModeIntercept::run()
{
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    if (!AP::seeker()->is_valid((uint32_t)timeout_ms.get())) {
        run_position_hold();
    } else {
        const AP_Seeker::State &st = AP::seeker()->get_state();

        // --- Yaw: null horizontal centroid error via yaw rate ---
        const float yaw_rate_rads = yaw_p.get() * st.centroid_x + yaw_d.get() * st.los_rate_x;
        auto_yaw.set_rate_rad(yaw_rate_rads);

        // --- Vertical: drive centroid_y to zero; compensate for pitch-tilt coupling ---
        const float body_accel_fwd = ahrs.get_accel().x;
        const float vel_up = vrt_p.get() * st.centroid_y + accel_comp.get() * body_accel_fwd;
        float vel_D = -vel_up;  // NED: positive = down
        const float zero_accel_D = 0.0f;
        pos_control->input_vel_accel_D_m(vel_D, zero_accel_D, false);

        // --- Forward: constant speed in current heading direction ---
        const float yaw_rad = ahrs.get_yaw();
        Vector2f vel_ne;
        vel_ne.x = speed.get() * cosf(yaw_rad);  // North
        vel_ne.y = speed.get() * sinf(yaw_rad);  // East
        Vector2f zero_accel_ne;
        pos_control->input_vel_accel_NE_m(vel_ne, zero_accel_ne, false);
    }

    // update position controllers
    pos_control->NE_update_controller();
    pos_control->D_update_controller();

    // call attitude controller
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

// Seeker timeout fallback: hold position with zero velocity
void ModeIntercept::run_position_hold()
{
    Vector2f vel_ne_zero;
    Vector2f accel_ne_zero;
    pos_control->input_vel_accel_NE_m(vel_ne_zero, accel_ne_zero, false);

    float vel_d_zero = 0.0f;
    pos_control->input_vel_accel_D_m(vel_d_zero, 0.0f, false);

    auto_yaw.set_rate_rad(0.0f);
}

#endif  // MODE_INTERCEPT_ENABLED
