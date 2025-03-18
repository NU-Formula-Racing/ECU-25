#include <Arduino.h>

#include "LUT.hpp"
#include "esp_can.h"
#include "fsm.hpp"
#include "inverter_driver.hpp"
#include "pins.hpp"
#include "throttle_brake_driver.hpp"
#include "virtualTimer.h"
//
void setup() { fsm_init(); }

void loop() { tick_timers(); }
