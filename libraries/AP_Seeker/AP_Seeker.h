#pragma once
#include <AP_HAL/AP_HAL.h>

class AP_Seeker {
public:
    struct State {
        float    los_rate_x;      // rad/s, positive = target moving right
        float    los_rate_y;      // rad/s, positive = target moving up
        float    centroid_x;      // fraction of FOV width  (-0.5…+0.5)
        float    centroid_y;      // fraction of FOV height (-0.5…+0.5)
        uint32_t last_update_ms;
        bool     target_found;
    };

    AP_Seeker() { _singleton = this; }

    // Called by each vehicle's GCS MAVLink handler
    void handle_seeker_target(const State &s) { _state = s; }

    // Returns true if target_found and packet arrived within timeout_ms
    bool is_valid(uint32_t timeout_ms) const;

    const State &get_state() const { return _state; }
    static AP_Seeker *get_singleton() { return _singleton; }

private:
    State _state{};
    static AP_Seeker *_singleton;
};

namespace AP { AP_Seeker *seeker(); }
