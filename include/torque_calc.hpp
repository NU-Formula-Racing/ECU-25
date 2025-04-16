#pragma once
#include <map>

#include "LUT_reference.hpp"
#include "lookup.hpp"

class TorqueCalc {
 public:
  TorqueCalc()
      : torqueLUTs(LUTs::IGBTTemp2Modifier_LUT, LUTs::BatteryTemp2Modifier_LUT,
                   LUTs::MotorTemp2Modifier_LUT, LUTs::Throttle2Modifier_LUT) {}

  int32_t calculate_accel_torque(int16_t igbt_temp, int16_t batt_temp, int16_t motor_temp,
                                 int16_t throttle);

 private:
  LUTs::TorqueLUTs torqueLUTs;

  // Max current/torque we can request from Inverter (in mA)
  // current:torque is ~1:1
  enum class TorqueReqLimit { kAccelMax = 80000, kRegenMax = 80000 };
  /* Functions */
  int32_t scale_torque(float torque, int32_t torque_max);

  // int16_t get_brake_modifier(int16_t brake_pressure);

  // float get_pump_duty_cycle(int16_t motor_temp, int16_t igbt_temp, int16_t batt_temp);

  // float get_fan_duty_cycle(int16_t coolant_temp);
};
