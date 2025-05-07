#pragma once

#include <iostream>
#include <map>

namespace LUT {

// Max current/torque we can request from Inverter (in mA)
// current:torque is ~1:1
enum class TorqueReqLimit { kAccelMax = 235000, kRegenMax = 0 };

enum class PWMLimit { kPumpMax = 255, kFanMax = 255 };

/* Power limit modifier LUTs */
// IGBT temp : Power limit modifier
extern const std::map<int16_t, float> IGBTTemp2Modifier_LUT;

// Battery temp : Power limit modifier
extern const std::map<int16_t, float> BatteryTemp2Modifier_LUT;

// Motor temp : Power limit modifier
extern const std::map<int16_t, float> MotorTemp2Modifier_LUT;

// Motor RPM : throttle %
extern const std::map<int16_t, float> RPM2Throttle_LUT;

// Throttle value : power limit modifier (Accel)
extern const std::map<int16_t, float> AccelThrottle2Modifier_LUT;

// Throttle value : power limit modifier (Regen)
extern const std::map<int16_t, float> RegenThrottle2Modifier_LUT;

/* Cooling LUTs */
// Motor temp : Pump duty cycle
extern const std::map<int16_t, float> MotorTemp2PumpDutyCycle_LUT;

// IGBT Temp : Pump duty cycle
extern const std::map<int16_t, float> IGBTTemp2PumpDutyCycle_LUT;

// Battery Temp : Pump duty cycle
extern const std::map<int16_t, float> BatteryTemp2PumpDutyCycle_LUT;

// Coolant Temp : Fan duty cycle
extern const std::map<int16_t, float> CoolantTemp2FanDutyCycle_LUT;

float lookup(int16_t key, const std::map<int16_t, float>& lut);

template <typename IntT>
IntT scale(float value, IntT max);

int16_t get_throttle_difference(int16_t real_throttle, int16_t throttle_max, int16_t motor_rpm);

std::pair<int32_t, int32_t> calculate_torque(int16_t throttle_difference);

int32_t calculate_accel_torque(int16_t igbt_temp, int16_t batt_temp, int16_t motor_temp,
                               int16_t throttle);

uint8_t calculate_pump_duty_cycle(int16_t motor_temp, int16_t igbt_temp, int16_t batt_temp);

uint8_t calculate_fan_duty_cycle(float coolant_temp);

}  // namespace LUT
