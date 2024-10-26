#include <Arduino.h>

#include "LUT.hpp"
#include "inverter_driver.hpp"
#include "esp_can.h"
#include "throttle_brake_driver.hpp"
#include "virtualTimer.h"
#include "pins.hpp"

enum BMSState 
{
  kShutdown = 0,
  kPrecharge = 1,
  kActive = 2,
  kCharging = 3,
  kFault = 4
};

enum BMSCommand
{
  NoAction = 0,
  PrechargeAndCloseContactors = 1,
  Shutdown = 2
};

enum ContactorsSwitch
{
  Open,
  Close
};

enum Drive_Lever_State
{
  Neutral,
  Drive
};

enum Brake_State
{
  NotPressed,
  PressedInNeutral
};

enum state
{
  OFF,
  N,
  DRIVE
};

// instantiate CAN bus
ESPCAN drive_bus{};

// instantiate timer group
VirtualTimerGroup timers;

// instantiate throttle/brake
ThrottleBrake throttle_brake{drive_bus};

// instantiate inverter
// Inverter inverter{drive_bus};

// Contactor Switch Variable (WILL BE CHANGED)
ContactorsSwitch contactor_switch = ContactorsSwitch::Open;

// State Variable for brake (WILL BE CHANGED)
bool is_brake_pressed = false;

// State Variable for Drive Lever (WILL BE CHANGED)
Drive_Lever_State drive_lever = Drive_Lever_State::Neutral;

// State Variable for brake pressed without drive switch
Brake_State brake_state = NotPressed;

// CAN signals -- get new addresses from DBC
// add rx: wheel speed
// add tx: 
// APPS1, APPS2, front brake, rear brake, torque request will be handled in their respective .hpp files
CANSignal<BMSState, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BMS_State{};
CANSignal<BMSCommand, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BMS_Command{};
CANSignal<float, 8, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(-40), false> batt_temp{};
CANSignal<uint8_t, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> current_state{};
CANRXMessage<2> BMS_message{drive_bus, 0x241, BMS_State, batt_temp};
CANTXMessage<1> BMS_command_message{drive_bus, 0x242, 8, 100, timers, BMS_Command};
CANTXMessage<1> Drive_status{drive_bus, 0x000, 8, 100, timers, current_state};

void change_state();
void process_state();

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

  // initialize inverter class -- registers rx messages from inverter
  // inverter.initialize();

  // initialize drive lever -- use interrupts & pinMode


}

void loop() {
  timers.Tick(millis());
  drive_bus.Tick();
}

void change_brake_state() {
  switch (brake_state) {
    case NotPressed:
      if (is_brake_pressed && drive_lever == Drive_Lever_State::Neutral) {
        brake_state = PressedInNeutral;
      }
      break;
    case PressedInNeutral:
      if (!is_brake_pressed) {
        brake_state = NotPressed;
      }
      break;
  }
}

void change_state() {
  change_brake_state();
  switch(current_state) {
    case OFF:
      if (contactor_switch == ContactorsSwitch::Close && BMS_State == BMSState::kActive) {
        current_state = N;
      }
      break;
    
    case N:
      if (brake == Brake_State::PressedInNeutral && drive_lever == Drive_Lever_State::Drive) {
        current_state = DRIVE;
      }
      else if (contactor_switch == ContactorsSwitch::Open || BMS_State == BMSState::kFault) {
        current_state = OFF;
      }
      break;

    case DRIVE:
      if (drive_lever == Drive_Lever_State::Neutral) {
        current_state = N;
      }
      else if (contactor_switch == ContactorsSwitch::Open || BMS_State == BMSState::kFault) {
        current_state = OFF;
      }
      break;
  }
}

void process_state() {
  switch(current_state) {
    case OFF:

    case N:

    case DRIVE:
    int x = 1;
  }
}