#pragma once
#include <map>

#include "lookup.hpp"

namespace LUTs {

struct TorqueLUTs {
  Lookup IGBTTemp2Modifier_LUT;
  Lookup BatteryTemp2Modifier_LUT;
  Lookup MotorTemp2Modifier_LUT;
  Lookup Throttle2Modifier_LUT;
  // Lookup BrakePressure2Modifier_LUT;

  TorqueLUTs(std::map<int16_t, float> igbt_temp2modifier,
             std::map<int16_t, float> battery_temp2modifier,
             std::map<int16_t, float> motor_temp2modifier,
             std::map<int16_t, float> throttle2modifier)
      : IGBTTemp2Modifier_LUT(Lookup(igbt_temp2modifier)),
        BatteryTemp2Modifier_LUT(Lookup(battery_temp2modifier)),
        MotorTemp2Modifier_LUT(Lookup(motor_temp2modifier)),
        Throttle2Modifier_LUT(Lookup(throttle2modifier)) {}
};

struct CoolingLUTs {
  Lookup MotorTemp2PumpDutyCycle_LUT;
  Lookup IGBTTemp2PumpDutyCycle_LUT;
  Lookup BatteryTemp2PumpDutyCycle_LUT;
  Lookup CoolantTemp2FanDutyCycle_LUT;

  CoolingLUTs(std::map<int16_t, float> motor_temp2pump_duty_cycle,
              std::map<int16_t, float> igbt_temp2pump_duty_cycle,
              std::map<int16_t, float> battery_temp2pump_duty_cycle,
              std::map<int16_t, float> coolant_temp2fan_duty_cycle)
      : MotorTemp2PumpDutyCycle_LUT(Lookup(motor_temp2pump_duty_cycle)),
        IGBTTemp2PumpDutyCycle_LUT(Lookup(igbt_temp2pump_duty_cycle)),
        BatteryTemp2PumpDutyCycle_LUT(Lookup(battery_temp2pump_duty_cycle)),
        CoolantTemp2FanDutyCycle_LUT(Lookup(coolant_temp2fan_duty_cycle)) {}
};

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
    {0, 0.0},     {102, 0.03},  {105, 0.09},  {307, 0.16},  {409, 0.23},  {512, 0.3},
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

}  // namespace LUTs
