#include "active_aero.hpp"

void ActiveAero::calculate_position(int16_t throttle, bool brake_pressed) {}

void ActiveAero::update_state() {
  if (enabled_ == ActiveAeroEnabled::kDisabled) {
    state_ = ActiveAeroState::kClosed;
    return;
  }

  if (enabled_ ==) }

void ActiveAero::update_can() {
  Active_Aero_State = state_;
  Active_Aero_Position = position_;
}