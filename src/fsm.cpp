#include <Arduino.h>

#include "LUT.hpp"
#include "inverter_driver.hpp"
#include "esp_can.h"
#include "throttle_brake_driver.hpp"
#include "virtualTimer.h"
#include "pins.hpp"
#include "fsm.hpp"


void fsm_init()
{
  Serial.begin(115200);
  Serial.println("fsm init");
  
  // initialize CAN bus
  drive_bus.Initialize(ESPCAN::BaudRate::kBaud1M);

  // initialize inverter class -- registers rx messages from inverter
  inverter.initialize();

  // initialize throttle/brake class -- should set up SPI transaction and set pinModes
  throttle_brake.initialize();

  // register BMS msg
  drive_bus.RegisterRXMessage(BMS_Message);

  // add change_state and process_state to timer group -- called every 10 ms
  timers.AddTimer(10, change_state);
  timers.AddTimer(10, process_state);
  Serial.println("added change and process to timergroup");

  // send and recieve inverter CAN messages
  // timers.AddTimer(10, read_inverter_CAN_wrapper);
  timers.AddTimer(10, send_inverter_CAN_wrapper);

  // send throttle/brake CAN messages -- can be less frequent since they're just going to logger
  // timers.AddTimer(100, send_throttle_brake_CAN_wrapper);

  // 10 ms timer for CAN messages
  timers.AddTimer(10, tick_CAN);

  // timer for print debugging msgs
  // timers.AddTimer(100, print_all);

  Serial.println("added everything to timergroup");

  // initialize state variables
  tsactive_switch = TSActive::Inactive;
  ready_to_drive = Ready_To_Drive_State::Neutral;
  Drive_State = State::OFF;

  // initialize dash switches
  initialize_dash_switches();
}

//// wrappers for send/read CAN functions: timers don't like if your callbacks are direct class member functions
// send inverter wrapper
static void send_inverter_CAN_wrapper() {
  inverter.send_inverter_CAN();
}
// read inverter wrapper
static void read_inverter_CAN_wrapper() {
  inverter.read_inverter_CAN();
}
// send throttle/brake wrapper
static void send_throttle_brake_CAN_wrapper() {
  throttle_brake.update_throttle_brake_CAN_signals();
}

// wrapper for CAN msgs sent/read every 10ms: inverter, fsm
void tick_CAN() {
  // Serial.println("tick fsm inverter CAN");
  // inverter.send_inverter_CAN();
  // inverter.read_inverter_CAN();
  Serial.println(millis());
  drive_bus.Tick();
}

// call this function when the ready to drive switch is flipped
// currently, the switch is active low
static void ready_to_drive_callback() {
  if (digitalRead(static_cast<uint8_t>(Pins::READY_TO_DRIVE_SWITCH)) == LOW && throttle_brake.is_brake_pressed()) {
    ready_to_drive = Ready_To_Drive_State::Drive;
  } else {
    ready_to_drive = Ready_To_Drive_State::Neutral;
  }
}

// call this function when the tsactive switch is flipped
// currently, the switch is active low
static void tsactive_callback() {
  if (digitalRead((uint8_t)Pins::TS_ACTIVE_PIN) == LOW) {
    tsactive_switch = TSActive::Active; 
    BMS_Command = BMSCommand::PrechargeAndCloseContactors;
  } else {
    tsactive_switch = TSActive::Inactive;
  }
}

static void initialize_dash_switches() {
  // initialize Ready To Drive Switch -- use interrupts & pinMode 
  pinMode((uint8_t)Pins::READY_TO_DRIVE_SWITCH, INPUT);
  attachInterrupt(digitalPinToInterrupt(static_cast<uint8_t>(Pins::READY_TO_DRIVE_SWITCH)), ready_to_drive_callback, CHANGE);

  // initialize tsactive switch -- use interrupts & pinMode
  pinMode((uint8_t)Pins::TS_ACTIVE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(static_cast<uint8_t>(Pins::TS_ACTIVE_PIN)), tsactive_callback, CHANGE);
}

// this function will be used to change the state of the vehicle based on the current state and the state of the switches
static void change_state() {
  switch(Drive_State) {
    case State::OFF:
      if (tsactive_switch == TSActive::Active && BMS_State == BMSState::kActive) {
        Drive_State = State::N;
      }
      break;
    
    case State::N:
      if (ready_to_drive == Ready_To_Drive_State::Drive) {
        Drive_State = State::DRIVE;
      }
      if (tsactive_switch == TSActive::Inactive || BMS_State == BMSState::kFault) {
        Drive_State = State::OFF;
      }
      break;

    case State::DRIVE:
      if (ready_to_drive == Ready_To_Drive_State::Neutral) {
        Drive_State = State::N;
      }
      if (tsactive_switch == TSActive::Inactive || BMS_State == BMSState::kFault) {
        Drive_State = State::OFF;
      }
      break;
  }
  // Serial.println("end of change state");
}

// this function will be used to calculate torque based on LUTs and traction control when its time
static void process_state() {
  switch(Drive_State) {
    case State::OFF:
      Serial.println("OFF");
      BMS_Command = BMSCommand::NoAction;
      inverter.request_torque(0);
      break;
    case State::N:
      Serial.println("N");
      BMS_Command = BMSCommand::NoAction;
      inverter.request_torque(0);
      break;
    case State::DRIVE:
      Serial.println("DRIVE");
      // int32_t torque_req = calculate_torque(); // use this function to calculate torque based on LUTs and traction control when its time
      int32_t torque_req;
      if (throttle_brake.is_implausibility_present()) {
        torque_req = 0;
      }
      else {
        torque_req = static_cast<int32_t>(throttle_brake.get_throttle());
      }
      inverter.request_torque(torque_req); 
      break;
  }
}

void print_fsm() {
  // Serial.print("Drive State: ");
  // Serial.println(static_cast<State>(Drive_State)); // cant print CAN signals ...
  // Serial.print("Ready to Drive: ");
  // Serial.println(static_cast<int>(ready_to_drive));
  // Serial.print("TS Active: ");
  // Serial.println(static_cast<int>(tsactive_switch));
  // Serial.print("BMS State: ");
  // Serial.println(static_cast<int>(BMS_State));
  // Serial.print("BMS Command: ");
  // Serial.println(static_cast<int>(BMS_Command));
}

void print_all() {
  print_fsm();
  inverter.print_inverter_info();
  throttle_brake.print_throttle_info();
}

void tick_timers() {
  // Serial.println("tick timers");
  timers.Tick(millis());
}

// CAN signals -- get new addresses from DBC
// add rx: wheel speed
// add tx: 
// APPS1, APPS2, front brake, rear brake, torque request will be handled in their respective .hpp files
CANSignal<BMSState, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BMS_State{};
CANSignal<BMSCommand, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BMS_Command{};
CANSignal<float, 8, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(-40), false> batt_temp{};
CANSignal<State, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> Drive_State{};
CANRXMessage<2> BMS_Message{drive_bus, 0x205, BMS_State, batt_temp}; // these addresses might be wrong, check DBC
CANTXMessage<1> BMS_Command_Message{drive_bus, 0x242, 8, 100, timers, BMS_Command};
CANTXMessage<1> Drive_Status{drive_bus, 0x206, 8, 100, timers, Drive_State};