#pragma once
#include <map>

#include "LUT_reference.hpp"
#include "lookup.hpp"

class CoolingCalc {
 public:
  CoolingCalc()
      : coolingLUTs(LUTs::MotorTemp2PumpDutyCycle_LUT, LUTs::IGBTTemp2PumpDutyCycle_LUT,
                    LUTs::BatteryTemp2PumpDutyCycle_LUT, LUTs::CoolantTemp2FanDutyCycle_LUT) {}

  // Get pump duty cycle based on motor, IGBT, and battery temperature
  float get_pump_duty_cycle(int16_t motor_temp, int16_t igbt_temp, int16_t batt_temp);

  // Get fan duty cycle based on coolant temperature
  float get_fan_duty_cycle(int16_t coolant_temp);

 private:
  LUTs::CoolingLUTs coolingLUTs;
};