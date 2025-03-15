#pragma once

#include <iostream>
#include <map>

// IGBT temp : Power limit modifier
extern std::map<uint16_t, float> IGBT_Temp_LUT;

// Battery temp : Power limit modifier
extern std::map<uint16_t, float> Battery_Temp_LUT;

// Motor temp : Power limit modifier
extern std::map<uint16_t, float> Motor_Temp_LUT;

// Motor RPM : Power limit modifier
extern std::map<uint16_t, float> Motor_RPM_LUT;

// Throttle value : Scaled power limit modifier (235 * power limit modifier)
extern std::map<uint16_t, float> Throttle_LUT;

// Brake % : float (when braking: factor to multiply by min(other LUTS) -- this product is the
// torque requested)
extern std::map<uint16_t, float> brakepercent2mAmp_LUT;