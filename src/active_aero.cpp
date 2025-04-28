#include "active_aero.hpp"

void ActiveAero::update_active_aero(int32_t set_current, bool brake_pressed) {
  // update CAN signals
  update_can();

  // closed: brake pressed or throttle < 25%
  // open: throttle > 25% and brake not pressed
  // only do all this if enabled and in drive
  if (enabled_ == ActiveAeroEnabled::kEnabled) {
    float current_percent =
        (static_cast<float>(set_current) / 235000.0f) * 100.0f;  // scale to 0-100%

    if (!brake_pressed && (current_percent > 25.0)) {
      position_ = static_cast<int16_t>(ActiveAeroPosition::kOpen);
      state_ = ActiveAeroState::kOpen;
    } else {
      position_ = static_cast<int16_t>(ActiveAeroPosition::kClosed);
      state_ = ActiveAeroState::kClosed;
    }
  }
}

void ActiveAero::update_can() {
  Active_Aero_State = state_;
  Active_Aero_Position = position_;
  enabled_ = Active_Aero_Enabled;
}