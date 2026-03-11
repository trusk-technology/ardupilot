#include "AP_Seeker.h"

AP_Seeker *AP_Seeker::_singleton;

bool AP_Seeker::is_valid(uint32_t timeout_ms) const
{
    if (!_state.target_found) {
        return false;
    }
    return (AP_HAL::millis() - _state.last_update_ms) < timeout_ms;
}

namespace AP {
AP_Seeker *seeker()
{
    return AP_Seeker::get_singleton();
}
}
