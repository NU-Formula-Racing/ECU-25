#include "torque_calc.hpp"

#include <map>

TorqueCalc::TorqueCalc() {
  LUTs::TorqueLUTs torqueLUTs(LUTs::IGBTTemp2Modifier_LUT, LUTs::BatteryTemp2Modifier_LUT,
                              LUTs::MotorTemp2Modifier_LUT, LUTs::Throttle2Modifier_LUT);
  this->torqueLUTs = torqueLUTs;
}

int32_t TorqueCalc::scale_torque(float torque, int32_t torque_max) {
  return static_cast<int32_t>(roundf(torque * static_cast<float>(torque_max)));
}

int32_t TorqueCalc::calculate_accel_torque(int16_t igbt_temp, int16_t batt_temp, int16_t motor_temp,
                                           int16_t throttle) {
  float igbt_mod = torqueLUTs.BatteryTemp2Modifier_LUT.lookup_val(igbt_temp);
  float batt_mod = torqueLUTs.BatteryTemp2Modifier_LUT.lookup_val(batt_temp);
  float motor_temp_mod = torqueLUTs.MotorTemp2Modifier_LUT.lookup_val(motor_temp);
  float throttle_mod = torqueLUTs.Throttle2Modifier_LUT.lookup_val(throttle);

  float mod_product = igbt_mod * batt_mod * motor_temp_mod * throttle_mod;

  return scale_torque(mod_product, static_cast<int32_t>(TorqueReqLimit::kAccelMax));
}

// int16_t get_brake_modifier(int16_t brake_pressure) {
// float brake_mod = lookup(brake_pressure, BrakePressure2Modifier_LUT);
//   return 0; // return scaled(brake_mod, 0, kCurrentLUTScaledMax);