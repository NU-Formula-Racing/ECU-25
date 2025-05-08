#include <cmath>
// #include <iostream>
#include <map>

#include "LUT.hpp"

namespace LUT {
// IGBT temp : Power limit modifier
const std::map<int16_t, float> IGBTTemp2Modifier_LUT{
    {0, 1.0},    {10, 1.0},   {20, 1.0},   {30, 1.0}, {40, 1.0},  {50, 1.0},
    {60, 1.0},   {70, 1.0},   {80, 1.0},   {90, 1.0}, {100, 1.0}, {110, 0.9},
    {120, 0.75}, {130, 0.25}, {140, 0.05}, {150, 0.0}};

// Battery temp : Power limit modifier
const std::map<int16_t, float> BatteryTemp2Modifier_LUT{
    {0, 1.0},  {5, 1.0},  {10, 1.0}, {15, 1.0},  {20, 1.0},  {25, 1.0}, {30, 1.0},
    {35, 1.0}, {40, 1.0}, {45, 1.0}, {50, 0.75}, {55, 0.25}, {60, 0.0},
};

// Motor temp : Power limit modifier
const std::map<int16_t, float> MotorTemp2Modifier_LUT{
    {0, 1.0},  {10, 1.0},  {20, 1.0},  {30, 1.0},  {40, 1.0},   {50, 1.0}, {60, 1.0},
    {70, 1.0}, {80, 0.95}, {90, 0.75}, {100, 0.2}, {110, 0.05}, {120, 0.0}};

// Motor RPM : throttle %
const std::map<int16_t, float> RPM2Throttle_LUT{
    {0, 0.0},      {200, 0.0},    {400, 0.07},   {600, 0.12},  {800, 0.16},
    {1000, 0.19},  {1200, 0.21},  {1400, 0.22},  {1600, 0.23}, {1800, 0.235},
    {2000, 0.239}, {2200, 0.244}, {2400, 0.247}, {2600, 0.25}, {10000, 0.25}};

// Throttle value : power limit modifier (Accel)
const std::map<int16_t, float> AccelThrottle2Modifier_LUT{
    {0, 0.0},     {102, 0.03},  {205, 0.09},  {307, 0.16},  {409, 0.23},  {512, 0.3},
    {614, 0.37},  {716, 0.44},  {819, 0.51},  {921, 0.58},  {1024, 0.65}, {1126, 0.72},
    {1228, 0.78}, {1331, 0.83}, {1433, 0.88}, {1535, 0.92}, {1638, 0.95}, {1740, 0.97},
    {1842, 0.98}, {1945, 0.99}, {2047, 1.0}};

// Throttle value : power limit modifier (Regen)
const std::map<int16_t, float> RegenThrottle2Modifier_LUT{
    {0, 0.0},     {102, 0.01},  {205, 0.02},  {307, 0.03},  {409, 0.04},  {512, 0.05},
    {614, 0.07},  {716, 0.11},  {819, 0.17},  {921, 0.24},  {1024, 0.32}, {1126, 0.43},
    {1228, 0.54}, {1331, 0.65}, {1433, 0.77}, {1535, 0.85}, {1638, 0.91}, {1740, 0.95},
    {1842, 0.97}, {1945, 0.99}, {2047, 1.0}};

// Motor temp : Pump duty cycle
const std::map<int16_t, float> MotorTemp2PumpDutyCycle_LUT{
    {0, 0.0},  {10, 0.0},  {20, 0.0}, {30, 0.0},  {40, 0.1},  {50, 0.25}, {60, 0.5},
    {70, 0.8}, {80, 0.95}, {90, 1.0}, {100, 1.0}, {110, 1.0}, {120, 1.0}};

// IGBT Temp : Pump duty cycle
const std::map<int16_t, float> IGBTTemp2PumpDutyCycle_LUT{
    {0, 0.0},   {10, 0.0}, {20, 0.0},  {30, 0.0},  {40, 0.0},  {50, 0.1},  {60, 0.3},  {70, 0.55},
    {80, 0.75}, {90, 0.9}, {100, 1.0}, {110, 1.0}, {120, 1.0}, {130, 1.0}, {140, 1.0}, {150, 1.0}};

// Battery Temp : Pump duty cycle
const std::map<int16_t, float> BatteryTemp2PumpDutyCycle_LUT{
    {0, 0.0},  {5, 0.0},  {10, 0.0}, {15, 0.0}, {20, 0.0}, {25, 0.0}, {30, 0.0},
    {35, 0.1}, {40, 0.4}, {45, 0.7}, {50, 0.9}, {55, 1.0}, {60, 1.0}};

// Coolant Temp : Fan duty cycle
const std::map<int16_t, float> CoolantTemp2FanDutyCycle_LUT{
    {0, 0.0},  {5, 0.0},  {10, 0.0}, {15, 0.0},  {20, 0.0}, {25, 0.05}, {30, 0.15},
    {35, 0.4}, {40, 0.7}, {45, 0.9}, {50, 0.97}, {55, 1.0}, {60, 1.0}};

float lookup(int16_t key, const std::map<int16_t, float>& lut) {
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
IntT scale(float value, IntT max) {
  return static_cast<IntT>(roundf(value * static_cast<float>(max)));
}

int16_t get_throttle_difference(int16_t real_throttle, int16_t throttle_max, int16_t motor_rpm) {
  float throttle_calc_float = lookup(motor_rpm, RPM2Throttle_LUT);
  int16_t throttle_calc = scale(throttle_calc_float, static_cast<int16_t>(throttle_max));

  return real_throttle - throttle_calc;
}

// <accel_mod, regen_mod>
std::pair<float, float> get_torque_mods(int16_t real_throttle, int16_t throttle_max,
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

float calculate_temp_mod(int16_t igbt_temp, int16_t batt_temp, int16_t motor_temp) {
  float igbt_mod = lookup(igbt_temp, IGBTTemp2Modifier_LUT);
  float batt_mod = lookup(batt_temp, BatteryTemp2Modifier_LUT);
  float motor_temp_mod = lookup(motor_temp, MotorTemp2Modifier_LUT);

  return igbt_mod * batt_mod * motor_temp_mod;
}

std::pair<int32_t, int32_t> calculate_torque_reqs(float temp_mod,
                                                  std::pair<float, float> torque_mods) {
  float accel_mod_product = temp_mod * torque_mods.first;
  float regen_mod_product = temp_mod * torque_mods.second;

  int32_t accel_torque =
      LUT::scale(accel_mod_product, static_cast<int32_t>(LUT::TorqueReqLimit::kAccelMax));
  int32_t regen_torque =
      LUT::scale(regen_mod_product, static_cast<int32_t>(LUT::TorqueReqLimit::kRegenMax));

  return std::make_pair(accel_torque, regen_torque);
}

uint8_t calculate_pump_duty_cycle(int16_t motor_temp, int16_t igbt_temp, int16_t batt_temp) {
  float motor_dc = lookup(motor_temp, MotorTemp2PumpDutyCycle_LUT);
  float igbt_dc = lookup(igbt_temp, IGBTTemp2PumpDutyCycle_LUT);
  float batt_dc = lookup(batt_temp, BatteryTemp2PumpDutyCycle_LUT);

  float dc_float = std::max(std::max(motor_dc, igbt_dc), batt_dc);

  return scale(dc_float, static_cast<uint8_t>(PWMLimit::kPumpMax));
  // take max of all three, have 2 sets of LUTs: 1 from 0
  // to "panic temp", 1 from "panic temp" to max
  // should be continuous between the modes
  // PDM just needs a msg with 2 signals: pump duty cycle and fan duty cycle, both scaled from 0-255
  // uint8_t
}

uint8_t calculate_fan_duty_cycle(float coolant_temp) {
  int16_t coolant_temp_int = static_cast<int16_t>(roundf(coolant_temp));

  float coolant_dc = lookup(coolant_temp_int, CoolantTemp2FanDutyCycle_LUT);

  return scale(coolant_dc, static_cast<uint8_t>(PWMLimit::kFanMax));
}
}  // namespace LUT