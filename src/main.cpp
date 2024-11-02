#include <Arduino.h>

#include "LUT.hpp"
#include "inverter_driver.hpp"
#include "esp_can.h"
#include "throttle_brake_driver.hpp"
#include "virtualTimer.h"
#include "pins.hpp"
#include "fsm.hpp"

void setup() {
  // Serial.begin(115200);
  // initialize CAN bus
  drive_bus.Initialize(ESPCAN::BaudRate::kBaud500K);

  // register BMS msg
  drive_bus.RegisterRXMessage(BMS_message);

  // add change_state and process_state to timer group -- called every 10 ms
  timers.AddTimer(10, change_state);
  timers.AddTimer(10, process_state);

  // initialize throttle/brake class -- should set up SPI transaction and set pinModes
  throttle_brake.initialize();

  tsactive_switch = TSActive::Inactive;
  ready_to_drive = Ready_To_Drive_State::Neutral;
  current_state = OFF;

  // initialize inverter class -- registers rx messages from inverter
  // inverter.initialize();

  // initialize Ready To Drive Switch -- use interrupts & pinMode
  pinMode((uint8_t)Pins::READY_TO_DRIVE_SWITCH, INPUT);
  attachInterrupt(digitalPinToInterrupt(static_cast<uint8_t>(Pins::READY_TO_DRIVE_SWITCH)), ready_to_drive_callback, CHANGE);

  // initialize tsactive switch -- use interrupts & pinMode
  pinMode((uint8_t)Pins::TS_ACTIVE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(static_cast<uint8_t>(Pins::TS_ACTIVE_PIN)), tsactive_callback, CHANGE);

}

// call this function when the ready to drive switch is flipped
// currently, the switch is active low
void ready_to_drive_callback() {
  if (digitalRead(static_cast<uint8_t>(Pins::READY_TO_DRIVE_SWITCH)) == LOW && throttle_brake.is_brake_pressed()) {
    ready_to_drive = Ready_To_Drive_State::Drive;
  } else {
    ready_to_drive = Ready_To_Drive_State::Neutral;
  }
}

// call this function when the tsactive switch is flipped
// currently, the switch is active low
void tsactive_callback() {
  if (digitalRead((uint8_t)Pins::TS_ACTIVE_PIN) == LOW) {
    tsactive_switch = TSActive::Active;
  } else {
    tsactive_switch = TSActive::Inactive;
  }
}

void change_state() {
  switch(current_state) {
    case OFF:
      if (tsactive_switch == TSActive::Active && BMS_State == BMSState::kActive) {
        current_state = N;
      }
      break;
    
    case N:
      if (ready_to_drive == Ready_To_Drive_State::Drive) {
        current_state = DRIVE;
      }
      if (tsactive_switch == TSActive::Inactive || BMS_State == BMSState::kFault) {
        current_state = OFF;
      }
      break;

    case DRIVE:
      if (ready_to_drive == Ready_To_Drive_State::Neutral) {
        current_state = N;
      }
      if (tsactive_switch == TSActive::Inactive || BMS_State == BMSState::kFault) {
        current_state = OFF;
      }
      break;
  }
}

void process_state() {
  switch(current_state) {
    case OFF:
    break;

    case N:
    break;

    case DRIVE:
    break;
    int x = 1;
  }
}

void loop() {
  timers.Tick(millis());
  drive_bus.Tick();
}