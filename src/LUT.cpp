#include "LUT.hpp"

#include <cmath>
#include <map>

void Lookup::updateCANLUTs() {
  RXLUT rxLUT = lut_can.processCAN();
  if (rxLUT.fileStatus == FileStatus::FILE_PRESENT_AND_VALID) {
    AccelThrottle2Modifier_LUT = rxLUT.lut;
    lut_can.setLUTIDResponse(rxLUT.LUTId);
  } else {
    lut_can.setLUTIDResponse(0);
    AccelThrottle2Modifier_LUT = DefaultAccelThrottle2Modifier_LUT;
  }
}

float Lookup::lookup(int16_t key, const std::map<int16_t, float>& lut) {
  auto it = lut.lower_bound(key);
  // if key is smaller than the smallest key, return the first value
  if (it == lut.begin()) {
    return it->second;
  }
  // if key is larger than the largest key, return the last value
  if (it == lut.end()) {
    return std::prev(it)->second;
  }
  // if key is exactly found in the LUT, return its value
  if (it->first == key) {
    return it->second;
  }
  // if we're here, key is btwn 2 entries in the LUT, interpolate
  auto upper = it;
  auto lower = std::prev(it);

  return lower->second + (upper->second - lower->second) * static_cast<float>(key - lower->first) /
                             static_cast<float>(upper->first - lower->first);
}

template <typename IntT>
IntT Lookup::scale(float value, IntT max) {
  return static_cast<IntT>(roundf(value * static_cast<float>(max)));
}

int16_t Lookup::get_throttle_difference(int16_t real_throttle, int16_t throttle_max,
                                        int16_t motor_rpm) {
  float throttle_calc_float = lookup(motor_rpm, RPM2Throttle_LUT);
  int16_t throttle_calc = scale(throttle_calc_float, static_cast<int16_t>(throttle_max));

  return real_throttle - throttle_calc;
}

// <accel_mod, regen_mod>
std::pair<float, float> Lookup::get_torque_mods(int16_t real_throttle, int16_t throttle_max,
                                                int16_t motor_rpm, bool brake_pressed) {
  int16_t throttle_diff = get_throttle_difference(real_throttle, throttle_max, motor_rpm);

  float accel_mod = 0.0f;
  float regen_mod = 0.0f;

  if (throttle_diff > 0 && !brake_pressed) {
    accel_mod = lookup(throttle_diff, AccelThrottle2Modifier_LUT);
    regen_mod = 0.0f;
  } else if (throttle_diff < 0 && !brake_pressed) {
    accel_mod = 0.0f;
    regen_mod = lookup(-throttle_diff, RegenThrottle2Modifier_LUT);
  } else {
    accel_mod = 0.0f;
    regen_mod = 0.0f;
  }

  return std::make_pair(accel_mod, regen_mod);
}

Lookup::TempLimitingType Lookup::is_temp_limiting(float temp_mod) {
  if (temp_mod < 1.0f) {
    return Lookup::TempLimitingType::kLimiting;
  } else {
    return Lookup::TempLimitingType::kNotLimiting;
  }
}

float Lookup::calculate_temp_mod(int16_t igbt_temp, int16_t batt_temp, int16_t motor_temp) {
  float igbt_mod = lookup(igbt_temp, IGBTTemp2Modifier_LUT);
  float batt_mod = lookup(batt_temp, BatteryTemp2Modifier_LUT);
  float motor_temp_mod = lookup(motor_temp, MotorTemp2Modifier_LUT);
  std::vector<float> temp_mods{igbt_mod, batt_mod, motor_temp_mod};

  for (int i = 0; i < temp_mods.size(); i++) {
    Lookup::TempLimitingType temp_limiting_status = is_temp_limiting(temp_mods.at(i));
    temp_limiting_statuses.at(i) = temp_limiting_status;
  }

  return igbt_mod * batt_mod * motor_temp_mod;
}

void Lookup::update_temp_limiting_status_CAN() {
  IGBT_Temp_Limiting = static_cast<bool>(temp_limiting_statuses.at(0));
  Battery_Temp_Limiting = static_cast<bool>(temp_limiting_statuses.at(1));
  Motor_Temp_Limiting = static_cast<bool>(temp_limiting_statuses.at(2));
}

std::pair<int32_t, int32_t> Lookup::calculate_torque_reqs(float temp_mod,
                                                          std::pair<float, float> torque_mods) {
  float accel_mod_product = temp_mod * torque_mods.first;
  float regen_mod_product = temp_mod * torque_mods.second;

  int32_t accel_torque =
      scale(accel_mod_product, static_cast<int32_t>(Lookup::TorqueReqLimit::kAccelMax));
  int32_t regen_torque =
      scale(regen_mod_product, static_cast<int32_t>(Lookup::TorqueReqLimit::kRegenMax));

  return std::make_pair(accel_torque, regen_torque);
}

uint8_t Lookup::calculate_pump_duty_cycle(int16_t motor_temp, int16_t igbt_temp,
                                          int16_t batt_temp) {
  float motor_dc = lookup(motor_temp, MotorTemp2PumpDutyCycle_LUT);
  float igbt_dc = lookup(igbt_temp, IGBTTemp2PumpDutyCycle_LUT);
  float batt_dc = lookup(batt_temp, BatteryTemp2PumpDutyCycle_LUT);

  float dc_float = std::max(std::max(motor_dc, igbt_dc), batt_dc);

  return scale(dc_float, static_cast<uint8_t>(PWMLimit::kPumpMax));
}

uint8_t Lookup::calculate_fan_duty_cycle(float coolant_temp) {
  int16_t coolant_temp_int = static_cast<int16_t>(roundf(coolant_temp));

  float coolant_dc = lookup(coolant_temp_int, CoolantTemp2FanDutyCycle_LUT);

  return scale(coolant_dc, static_cast<uint8_t>(PWMLimit::kFanMax));
}
