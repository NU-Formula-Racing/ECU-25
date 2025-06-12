#pragma once

#include <iostream>
#include <map>

#include "can_interface.h"
#include "esp_can.h"
#include "lut_can.hpp"
#include "virtualTimer.h"

class Lookup {
 public:
  Lookup(ICAN& can_interface, VirtualTimerGroup& timers)
      : can_interface(can_interface), timers(timers) {};
  // Max current/torque we can request from Inverter (in mA)
  // current:torque is ~1:1
  enum class TorqueReqLimit { kAccelMax = 235000, kRegenMax = 235000 };

  enum class PWMLimit { kPumpMax = 255, kFanMax = 255 };

  void updateCANLUTs();

  void update_status_CAN();

  float lookup(int16_t key, const std::map<int16_t, float>& lut);

  template <typename IntT>
  IntT scale(float value, IntT max);

  int16_t get_throttle_index(int16_t real_throttle, int16_t throttle_max, int16_t motor_rpm);

  // returns <accel_mod, regen_mod>
  std::pair<float, float> get_torque_mods(int16_t real_throttle, int16_t throttle_max,
                                          int16_t motor_rpm, bool brake_pressed);

  float calculate_temp_mod(int16_t igbt_temp, int16_t batt_temp, int16_t motor_temp);

  // returns <accel_torque, regen_torque>
  std::pair<int32_t, int32_t> calculate_torque_reqs(float temp_mod,
                                                    std::pair<float, float> torque_mods);

  uint8_t calculate_pump_duty_cycle(int16_t motor_temp, int16_t igbt_temp, int16_t batt_temp);

  uint8_t calculate_fan_duty_cycle(float coolant_temp);

 private:
  ICAN& can_interface;
  VirtualTimerGroup& timers;

  LUTCan lut_can{can_interface, timers};

  enum class TempLimitingType { kNotLimiting = 0, kLimiting = 1 };
  enum class TorqueStatusType { kZero = 0, kAccel = 1, kRegen = 2 };

  struct CANData {
    TorqueStatusType torque_status;
    std::vector<TempLimitingType> temp_limiting_statuses;
  };

  CANData can_data{
      .torque_status = TorqueStatusType::kZero,
      .temp_limiting_statuses = {TempLimitingType::kNotLimiting, TempLimitingType::kNotLimiting,
                                 TempLimitingType::kNotLimiting}};

  Lookup::TempLimitingType is_temp_limiting(float temp_mod);

  CANSignal<bool, 0, 1, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      IGBT_Temp_Limiting{};
  CANSignal<bool, 1, 1, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      Battery_Temp_Limiting{};
  CANSignal<bool, 2, 1, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      Motor_Temp_Limiting{};
  CANTXMessage<3> ECU_Temp_Limiting_Status{
      can_interface,      0x20B, 1, 100, timers, IGBT_Temp_Limiting, Battery_Temp_Limiting,
      Motor_Temp_Limiting};

  CANSignal<uint8_t, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      Torque_Status{};
  CANTXMessage<1> ECU_Torque_Status{can_interface, 0x20C, 1, 100, timers, Torque_Status};

  /* Power limit modifier LUTs */
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
  const std::map<int16_t, float> DefaultAccelThrottle2Modifier_LUT{
      {0, 0.0},     {102, 0.03},  {205, 0.09},  {307, 0.16},  {409, 0.23},  {512, 0.3},
      {614, 0.37},  {716, 0.44},  {819, 0.51},  {921, 0.58},  {1024, 0.65}, {1126, 0.72},
      {1228, 0.78}, {1331, 0.83}, {1433, 0.88}, {1535, 0.92}, {1638, 0.95}, {1740, 0.97},
      {1842, 0.98}, {1945, 0.99}, {2047, 1.0}};

  std::map<int16_t, float> AccelThrottle2Modifier_LUT = DefaultAccelThrottle2Modifier_LUT;

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
      {0, 0.0},   {10, 0.0},  {20, 0.0},  {30, 0.0}, {40, 0.0},  {50, 0.1},
      {60, 0.3},  {70, 0.55}, {80, 0.75}, {90, 0.9}, {100, 1.0}, {110, 1.0},
      {120, 1.0}, {130, 1.0}, {140, 1.0}, {150, 1.0}};

  // Battery Temp : Pump duty cycle
  const std::map<int16_t, float> BatteryTemp2PumpDutyCycle_LUT{
      {0, 0.0},  {5, 0.0},  {10, 0.0}, {15, 0.0}, {20, 0.0}, {25, 0.0}, {30, 0.0},
      {35, 0.1}, {40, 0.4}, {45, 0.7}, {50, 0.9}, {55, 1.0}, {60, 1.0}};

  // Coolant Temp : Fan duty cycle
  const std::map<int16_t, float> CoolantTemp2FanDutyCycle_LUT{
      {0, 0.0},  {5, 0.0},  {10, 0.0}, {15, 0.0},  {20, 0.0}, {25, 0.05}, {30, 0.15},
      {35, 0.4}, {40, 0.7}, {45, 0.9}, {50, 0.97}, {55, 1.0}, {60, 1.0}};
};
