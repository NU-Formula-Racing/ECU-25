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

// Throttle value : Scaled power limit modifier (235 * power limit modifier)
const std::map<int16_t, float> Throttle2Modifier_LUT{
    {0, 0.0},     {102, 0.03},  {205, 0.09},  {307, 0.16},  {409, 0.23},  {512, 0.3},
    {614, 0.37},  {716, 0.44},  {819, 0.51},  {921, 0.58},  {1024, 0.65}, {1126, 0.72},
    {1228, 0.78}, {1331, 0.83}, {1433, 0.88}, {1535, 0.92}, {1638, 0.95}, {1740, 0.97},
    {1842, 0.98}, {1945, 0.99}, {2047, 1.0}};

// Brake pressure : Power limit modifier
// const std::map<int16_t, float> BrakePressure2Modifier_LUT{};

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

int32_t scale_torque(float torque, int32_t torque_max) {
  return static_cast<int32_t>(roundf(torque * static_cast<float>(torque_max)));
}

int32_t calculate_accel_torque(int16_t igbt_temp, int16_t batt_temp, int16_t motor_temp,
                               int16_t throttle) {
  float igbt_mod = lookup(igbt_temp, IGBTTemp2Modifier_LUT);
  float batt_mod = lookup(batt_temp, BatteryTemp2Modifier_LUT);
  float motor_temp_mod = lookup(motor_temp, MotorTemp2Modifier_LUT);
  float throttle_mod = lookup(throttle, Throttle2Modifier_LUT);

  float mod_product = igbt_mod * batt_mod * motor_temp_mod * throttle_mod;

  return LUT::scale_torque(mod_product, static_cast<int32_t>(LUT::TorqueReqLimit::kAccelMax));
}

// int16_t get_brake_modifier(int16_t brake_pressure) {
// float brake_mod = lookup(brake_pressure, BrakePressure2Modifier_LUT);
//   return 0; // return scaled(brake_mod, 0, kCurrentLUTScaledMax);

float get_pump_duty_cycle(int16_t motor_temp, int16_t igbt_temp, int16_t batt_temp) {
  float motor_dc = lookup(motor_temp, MotorTemp2PumpDutyCycle_LUT);
  float igbt_dc = lookup(igbt_temp, IGBTTemp2PumpDutyCycle_LUT);
  float batt_dc = lookup(batt_temp, BatteryTemp2PumpDutyCycle_LUT);

  return motor_dc * igbt_dc * batt_dc;  // take max of all three, have 2 sets of LUTs: 1 from 0 to
                                        // "panic temp", 1 from "panic temp" to max
  // should be continuous between the modes
  // PDM just needs a msg with 2 signals: pump duty cycle and fan duty cycle, both scaled from 0-255
  // uint8_t
}

float get_fan_duty_cycle(int16_t coolant_temp) {
  float coolant_dc =
      lookup(coolant_temp, CoolantTemp2FanDutyCycle_LUT);  // also get a wheelspeed : fan duty cycle
                                                           // LUT. faster wheelspeed = slower fan
  return coolant_dc;
}
}  // namespace LUT