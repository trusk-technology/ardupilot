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

    // initialise horizontal speed and acceleration limits (4.6.x API: cms units)
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialise vertical speed and acceleration limits (4.6.x API: cms units)
    pos_control->set_max_speed_accel_z(-wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(-wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise position controllers
    pos_control->init_xy_controller();
    pos_control->init_z_controller();

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
        // 4.6.x: set_rate takes centidegrees/s; convert rad/s → cds
        const float yaw_rate_rads = yaw_p.get() * st.centroid_x + yaw_d.get() * st.los_rate_x;
        auto_yaw.set_mode(AutoYaw::Mode::RATE);
        auto_yaw.set_rate(degrees(yaw_rate_rads) * 100.0f);

        // --- Vertical: drive centroid_y to zero; compensate for pitch-tilt coupling ---
        // 4.6.x: input_vel_accel_z uses cm/s, Z positive = down (NED)
        const float body_accel_fwd = ahrs.get_accel().x;
        const float vel_up_ms = vrt_p.get() * st.centroid_y + accel_comp.get() * body_accel_fwd;
        float vel_z_cms = -vel_up_ms * 100.0f;  // up→down sign flip, m→cm
        const float zero_accel_z = 0.0f;
        pos_control->input_vel_accel_z(vel_z_cms, zero_accel_z, false);

        // --- Forward: constant speed in current heading direction ---
        // 4.6.x: input_vel_accel_xy uses cm/s
        const float yaw_rad = ahrs.get_yaw();
        Vector2f vel_xy_cms;
        vel_xy_cms.x = speed.get() * 100.0f * cosf(yaw_rad);  // North cm/s
        vel_xy_cms.y = speed.get() * 100.0f * sinf(yaw_rad);  // East  cm/s
        Vector2f zero_accel_xy;
        pos_control->input_vel_accel_xy(vel_xy_cms, zero_accel_xy, false);
    }

    // update position controllers
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

// Seeker timeout fallback: hold position with zero velocity
void ModeIntercept::run_position_hold()
{
    // 4.6.x API: input_vel_accel_xy / input_vel_accel_z use cm/s
    Vector2f vel_xy_zero;
    Vector2f accel_xy_zero;
    pos_control->input_vel_accel_xy(vel_xy_zero, accel_xy_zero, false);

    float vel_z_zero = 0.0f;
    pos_control->input_vel_accel_z(vel_z_zero, 0.0f, false);

    auto_yaw.set_mode(AutoYaw::Mode::HOLD);
}

#endif  // MODE_INTERCEPT_ENABLED
