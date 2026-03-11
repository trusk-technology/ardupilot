#include "mode.h"
#include "Plane.h"

const AP_Param::GroupInfo ModeIntercept::var_info[] = {
    // @Param: THR
    // @DisplayName: Intercept throttle
    // @Description: Fixed throttle percentage used in INTERCEPT mode.
    // @Range: 0 100
    // @Units: %
    AP_GROUPINFO("THR",  0, ModeIntercept, throttle,   50.0f),

    // @Param: ROLL
    // @DisplayName: Intercept roll gain
    // @Description: Gain converting seeker X-axis LOS rate to roll demand (centi-deg per rad/s).
    // @Range: 0 10000
    AP_GROUPINFO("ROLL", 1, ModeIntercept, roll_gain,  1000.0f),

    // @Param: PTCH
    // @DisplayName: Intercept pitch gain
    // @Description: Gain converting seeker Y-axis LOS rate to pitch demand (centi-deg per rad/s).
    // @Range: 0 10000
    AP_GROUPINFO("PTCH", 2, ModeIntercept, pitch_gain, 1000.0f),

    // @Param: ANG
    // @DisplayName: Seeker mounting angle
    // @Description: Angle offset of the seeker from the aircraft forward axis (degrees). 0 = straight ahead.
    // @Range: -90 90
    // @Units: deg
    AP_GROUPINFO("ANG",  3, ModeIntercept, seeker_ang,  0.0f),

    // @Param: TOUT
    // @DisplayName: Seeker data timeout
    // @Description: Time in milliseconds after which missing SEEKER_TARGET messages cause the mode to level off.
    // @Range: 100 5000
    // @Units: ms
    AP_GROUPINFO("TOUT", 4, ModeIntercept, timeout_ms, 500.0f),

    AP_GROUPEND
};

bool ModeIntercept::_enter()
{
    // Reset seeker state so stale data doesn't immediately drive commands
    plane.seeker_state.target_found = false;
    return true;
}

void ModeIntercept::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    const bool data_valid = plane.seeker_state.target_found &&
                            (now_ms - plane.seeker_state.last_update_ms) < (uint32_t)timeout_ms.get();

    if (data_valid) {
        // Null X LOS rate by rolling, Y LOS rate by pitching
        plane.nav_roll_cd  = (int32_t)(plane.seeker_state.los_rate_x * roll_gain.get());
        plane.nav_pitch_cd = (int32_t)(plane.seeker_state.los_rate_y * pitch_gain.get());
    } else {
        // No valid seeker data — fly straight and level
        plane.nav_roll_cd  = 0;
        plane.nav_pitch_cd = 0;
    }

    // Constrain to vehicle limits
    plane.nav_roll_cd  = constrain_int32(plane.nav_roll_cd,
                                         -plane.roll_limit_cd,
                                          plane.roll_limit_cd);
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd,
                                          plane.pitch_limit_min * 100,
                                          plane.aparm.pitch_limit_max.get() * 100);

    // Fixed throttle — bypass TECS entirely
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle.get());
}
