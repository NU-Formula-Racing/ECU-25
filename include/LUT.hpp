#pragma once

#include <iostream>
#include <map>

// IGBT temp : milliAmp
std::map<uint16_t, int16_t> IGBTtemp2amp_LUT{
    {0, 0},
    {1,1},
    // get LUT values from Benji sims
}

// Battery temp : milliAmp
std::map<uint16_t, int16_t> batterytemp2amp_LUT{
    {0, 0},
    {1,1},
    // get LUT values from Benji sims
}

// Motor temp : milliAmp
std::map<uint16_t, int16_t> motortemp2amp_LUT{
    {0, 0},
    {1,1},
    // get LUT values from Benji sims
}

// Motor RPM : milliAmp
std::map<uint16_t, int16_t> motorRPM2amp_LUT{
    {0, 0},
    {1,1},
    // get LUT values from Benji sims
}

// Throttle % : float (factor to multiply by min(other LUTS) -- this product is the torque requested)
std::map<uint16_t, int16_t> throttlepercent2amp_LUT{
    {0, 0},
    {1000,10.4},
    // get LUT values from Benji sims
}