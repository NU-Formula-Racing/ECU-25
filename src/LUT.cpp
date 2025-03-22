#include <iostream>
#include <map>

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
    {0, 0.0},    {5, 7.05},    {10, 21.15}, {15, 37.6},   {20, 54.05}, {25, 70.5},   {30, 86.95},
    {35, 103.4}, {40, 119.85}, {45, 136.3}, {50, 152.75}, {55, 169.2}, {60, 183.3},  {65, 195.05},
    {70, 206.8}, {75, 216.2},  {80, 223.5}, {85, 227.95}, {90, 230.3}, {95, 232.65}, {100, 235.0}};

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

  return lower->second +
         (upper->second - lower->second) * (key - lower->first) / (upper->first - lower->first);
}

float get_modifier(int16_t igbt_temp, int16_t batt_temp, int16_t motor_temp, int16_t throttle) {
  float igbt_mod = lookup(igbt_temp, IGBTTemp2Modifier_LUT);
  float batt_mod = lookup(batt_temp, BatteryTemp2Modifier_LUT);
  float motor_temp_mod = lookup(motor_temp, MotorTemp2Modifier_LUT);
  float throttle_mod = lookup(throttle, Throttle2Modifier_LUT);

  return igbt_mod * batt_mod * motor_temp_mod * throttle_mod;
}

float get_pump_duty_cycle(int16_t motor_temp, int16_t igbt_temp, int16_t batt_temp) {
  float motor_dc = lookup(motor_temp, MotorTemp2PumpDutyCycle_LUT);
  float igbt_dc = lookup(igbt_temp, IGBTTemp2PumpDutyCycle_LUT);
  float batt_dc = lookup(batt_temp, BatteryTemp2PumpDutyCycle_LUT);

  return motor_dc * igbt_dc * batt_dc;
}

float get_fan_duty_cycle(int16_t coolant_temp) {
  float coolant_dc = lookup(coolant_temp, CoolantTemp2FanDutyCycle_LUT);
  return coolant_dc;
}
}  // namespace LUT