#pragma once

#include <iostream>
#include <map>

namespace LUT {
/* Power limit modifier LUTs */
// IGBT temp : Power limit modifier
extern const std::map<int16_t, float> IGBTTemp2Modifier_LUT;

// Battery temp : Power limit modifier
extern const std::map<int16_t, float> BatteryTemp2Modifier_LUT;

// Motor temp : Power limit modifier
extern const std::map<int16_t, float> MotorTemp2Modifier_LUT;

// Throttle value : Scaled power limit modifier (235 * power limit modifier)
extern const std::map<int16_t, float> Throttle2Modifier_LUT;

/* Cooling LUTs */
// Motor temp : Pump duty cycle
extern const std::map<int16_t, float> MotorTemp2PumpDutyCycle_LUT;

// IGBT Temp : Pump duty cycle
extern const std::map<int16_t, float> IGBTTemp2PumpDutyCycle_LUT;

// Battery Temp : Pump duty cycle
extern const std::map<int16_t, float> BatteryTemp2PumpDutyCycle_LUT;

// Coolant Temp : Fan duty cycle
extern const std::map<int16_t, float> CoolantTemp2FanDutyCycle_LUT;

/* Functions */
float lookup(int16_t key, const std::map<int16_t, float>& lut);

float get_modifier(int16_t igbt_temp, int16_t batt_temp, int16_t motor_temp, int16_t throttle);

float get_pump_duty_cycle(int16_t motor_temp, int16_t igbt_temp, int16_t batt_temp);

float get_fan_duty_cycle(int16_t coolant_temp);

}  // namespace LUT
