#pragma once

#include <iostream>
#include <map>

// IGBT temp : milliAmp
extern std::map<uint16_t, float> IGBTtemp2mAmp_LUT;

// Battery temp : milliAmp
extern std::map<uint16_t, float> batterytemp2mAmp_LUT;

// Motor temp : milliAmp
extern std::map<uint16_t, float> motortemp2mAmp_LUT;

// Motor RPM : milliAmp
extern std::map<uint16_t, float> motorRPM2mAmp_LUT;

// Throttle % : float (when accelerating: factor to multiply by min(other LUTS) -- this product is
// the torque requested)
extern std::map<uint16_t, float> throttlepercent2mAmp_LUT;

// Brake % : float (when braking: factor to multiply by min(other LUTS) -- this product is the
// torque requested)
extern std::map<uint16_t, float> brakepercent2mAmp_LUT;