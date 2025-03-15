#include <iostream>
#include <map>

// IGBT temp : Power limit modifier
std::map<uint16_t, float> IGBT_Temp_LUT{{0, 1.0},    {10, 1.0},   {20, 1.0},   {30, 1.0},
                                        {40, 1.0},   {50, 1.0},   {60, 1.0},   {70, 1.0},
                                        {80, 1.0},   {90, 1.0},   {100, 1.0},  {110, 0.9},
                                        {120, 0.75}, {130, 0.25}, {140, 0.05}, {150, 0.0}};

// Battery temp : Power limit modifier
std::map<uint16_t, float> Battery_Temp_LUT{
    {0, 1.0},  {5, 1.0},  {10, 1.0}, {15, 1.0},  {20, 1.0},  {25, 1.0}, {30, 1.0},
    {35, 1.0}, {40, 1.0}, {45, 1.0}, {50, 0.75}, {55, 0.25}, {60, 0.0},
};

// Motor temp : Power limit modifier
std::map<uint16_t, float> Motor_Temp_LUT{{0, 1.0},   {10, 1.0},   {20, 1.0}, {30, 1.0},  {40, 1.0},
                                         {50, 1.0},  {60, 1.0},   {70, 1.0}, {80, 0.95}, {90, 0.75},
                                         {100, 0.2}, {110, 0.05}, {120, 0.0}};

// Motor RPM : Power limit modifier
std::map<uint16_t, float> Motor_RPM_LUT{
    {0, 0}, {1, 1},
    // get LUT values from Benji
};

// Throttle value : Scaled power limit modifier (235 * power limit modifier)
std::map<uint16_t, float> Throttle_LUT{
    {0, 0.0},    {5, 7.05},    {10, 21.15}, {15, 37.6},   {20, 54.05}, {25, 70.5},   {30, 86.95},
    {35, 103.4}, {40, 119.85}, {45, 136.3}, {50, 152.75}, {55, 169.2}, {60, 183.3},  {65, 195.05},
    {70, 206.8}, {75, 216.2},  {80, 223.5}, {85, 227.95}, {90, 230.3}, {95, 232.65}, {100, 235.0}};

// Brake % : float (when braking: factor to multiply by min(other LUTS) -- this product is the
// torque requested)
std::map<uint16_t, float> brakepercent2mAmp_LUT{
    {0, 0}, {1000, 10.4},
    // get LUT values from Benji
};