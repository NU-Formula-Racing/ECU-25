#include "cooling_calc.hpp"

#include <map>

CoolingCalc::CoolingCalc() {
  LUTs::CoolingLUTs coolingLUTs(
      LUTs::IGBTTemp2PumpDutyCycle_LUT, LUTs::BatteryTemp2PumpDutyCycle_LUT,
      LUTs::MotorTemp2PumpDutyCycle_LUT, LUTs::CoolantTemp2FanDutyCycle_LUT);
  this->coolingLUTs = coolingLUTs;
}

float CoolingCalc::get_pump_duty_cycle(int16_t motor_temp, int16_t igbt_temp, int16_t batt_temp) {
  float motor_dc = coolingLUTs.MotorTemp2PumpDutyCycle_LUT.lookup_val(motor_temp);
  float igbt_dc = coolingLUTs.IGBTTemp2PumpDutyCycle_LUT.lookup_val(igbt_temp);
  float batt_dc = coolingLUTs.BatteryTemp2PumpDutyCycle_LUT.lookup_val(batt_temp);

  return motor_dc * igbt_dc * batt_dc;
}

float CoolingCalc::get_fan_duty_cycle(int16_t coolant_temp) {
  float coolant_dc = coolingLUTs.CoolantTemp2FanDutyCycle_LUT.lookup_val(coolant_temp);
  return coolant_dc;
}