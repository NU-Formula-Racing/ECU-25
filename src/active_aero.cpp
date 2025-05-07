#include "active_aero.hpp"

void ActiveAero::update_active_aero(int32_t set_current, float max_current, bool brake_pressed) {
  // update CAN signals
  update_can();

  // closed: brake pressed or throttle < 25%
  // open: throttle > 25% and brake not pressed
  // only do all this if enabled and in drive
  if (enabled == ActiveAeroEnabled::kEnabled) {
    float current_percent =
        (static_cast<float>(set_current) / max_current) * 100.0f;  // scale to 0-100%

    if (!brake_pressed && (current_percent > 25.0)) {
      position = static_cast<int16_t>(ActiveAeroPosition::kOpen);
      state = ActiveAeroState::kOpen;
    } else {
      position = static_cast<int16_t>(ActiveAeroPosition::kClosed);
      state = ActiveAeroState::kClosed;
    }
  }
}

void ActiveAero::update_can() {
  Active_Aero_State = state;
  Active_Aero_Position = position;
  enabled = Active_Aero_Enabled;
}