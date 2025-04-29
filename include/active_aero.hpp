#pragma once

#ifdef ESP32
#include "esp_can.h"
#endif
#include "can_interface.h"
#include "throttle_brake_driver.hpp"
#include "virtualTimer.h"

enum class ActiveAeroState {
  kClosed = 0,
  kOpen = 1,
};

enum class ActiveAeroEnabled {
  kEnabled = 0,
  kDisabled = 1,
};

enum class ActiveAeroPosition {  // update these with servo positions once we know them
  kClosed = 1000,
  kOpen = 2000,
};

class ActiveAero {
 public:
  ActiveAero(ICAN& can_interface_, VirtualTimerGroup& timer_group)
      : can_interface(can_interface_), timers(timer_group) {};

  void update_active_aero(int32_t set_current, float max_current, bool brake_pressed);

 private:
  void update_can();

  ActiveAeroState state_ = ActiveAeroState::kClosed;
  ActiveAeroEnabled enabled_ = ActiveAeroEnabled::kEnabled;
  int16_t position_;

  VirtualTimerGroup& timers;

  // CAN
  ICAN& can_interface;

  CANSignal<ActiveAeroState, 0, 1, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      Active_Aero_State{};
  CANSignal<int16_t, 1, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      Active_Aero_Position{};
  CANTXMessage<2> ECU_Active_Aero_Command{
      can_interface, 0x208, 4, 100, timers, Active_Aero_State, Active_Aero_Position};

  CANSignal<ActiveAeroEnabled, 0, 1, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      Active_Aero_Enabled{};
  CANRXMessage<1> Active_Aero_Enable{can_interface, 0x209, Active_Aero_Enabled};
};