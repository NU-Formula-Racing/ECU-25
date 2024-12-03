#include <Arduino.h>
#include "LUT.hpp"
#include "inverter_driver.hpp"

#include "esp_can.h"

#include "throttle_brake_driver.hpp"
#include "virtualTimer.h"
#include "pins.hpp"
#include "fsm.hpp"

void setup() {
  fsm_init();
}

void loop() {
  timers.Tick(millis());
  drive_bus.Tick();
}
