#include <iostream>
#include <map>

// IGBT temp : milliAmp
std::map<uint16_t, float> IGBTtemp2mAmp_LUT{
    {0, 0}, {1, 1},
    // get LUT values from Benji sims
};

// Battery temp : milliAmp
std::map<uint16_t, float> batterytemp2mAmp_LUT{
    {0, 0}, {1, 1},
    // get LUT values from Benji sims
};

// Motor temp : milliAmp
std::map<uint16_t, float> motortemp2mAmp_LUT{
    {0, 0}, {1, 1},
    // get LUT values from Benji sims
};

// Motor RPM : milliAmp
std::map<uint16_t, float> motorRPM2mAmp_LUT{
    {0, 0}, {1, 1},
    // get LUT values from Benji sims
};

// Throttle % : float (when accelerating: factor to multiply by min(other LUTS) -- this product is
// the torque requested)
std::map<uint16_t, float> throttlepercent2mAmp_LUT{
    {0, 0}, {1000, 10.4},
    // get LUT values from Benji sims
};

// Brake % : float (when braking: factor to multiply by min(other LUTS) -- this product is the
// torque requested)
std::map<uint16_t, float> brakepercent2mAmp_LUT{
    {0, 0}, {1000, 10.4},
    // get LUT values from Benji sims
};