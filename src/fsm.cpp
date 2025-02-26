#include "fsm.hpp"

#include <Arduino.h>

#include "LUT.hpp"
#include "esp_can.h"
#include "inverter_driver.hpp"
#include "pins.hpp"
#include "throttle_brake_driver.hpp"
#include "virtualTimer.h"

// initialize state variables
TSActive tsactive_switch;
Ready_To_Drive_State ready_to_drive;
Ready_To_Drive_State ready_to_drive_switch;

// instantiate CAN bus
ESPCAN drive_bus{};

// instantiate timer group
VirtualTimerGroup timers;

// instantiate throttle/brake
ThrottleBrake throttle_brake{drive_bus, timers, APPSs_disagree_timer, brake_implausible_timer};

// instantiate inverter
Inverter inverter{drive_bus, timers};

void fsm_init() {
  Serial.begin(115200);

  // initialize CAN bus
  drive_bus.Initialize(ESPCAN::BaudRate::kBaud500K);

  // initialize inverter class
  inverter.initialize();

  // initialize throttle/brake class -- should set up SPI transaction and set pinModes
  throttle_brake.initialize();

  // register BMS msg
  drive_bus.RegisterRXMessage(BMS_Status);

  // add change_state and process_state to timer group -- called every 10 ms
  timers.AddTimer(10, change_state);
  timers.AddTimer(10, process_state);

  // send and recieve inverter CAN messages
  timers.AddTimer(10, update_inverter);

  // refresh throttle/brake values and check for implausibilities
  timers.AddTimer(10, refresh_throttle_brake);

  // 10 ms timer for CAN messages
  timers.AddTimer(10, tick_CAN);

  // timer for print debugging msgs
  timers.AddTimer(1000, print_fsm);

  // initialize state variables
  tsactive_switch = TSActive::Inactive;
  ready_to_drive = Ready_To_Drive_State::Neutral;
  ready_to_drive_switch = Ready_To_Drive_State::Neutral;
  Drive_State = State::OFF;

  // initialize dash switches
  initialize_dash_switches();
}

//// wrappers for send/read CAN functions: timers don't like if your callbacks are direct class
/// member functions
// send inverter wrapper
void update_inverter() {
  inverter.read_inverter_CAN();
  inverter.send_inverter_CAN();
}

// wrapper for CAN msgs sent/read every 10ms: inverter, fsm
void tick_CAN() { drive_bus.Tick(); }

// read from ADCs, update internal throttle/brake values, and perform internal implausibility checks
void refresh_throttle_brake() {
  throttle_brake.update_sensor_values();
  throttle_brake.check_for_implausibilities();
  throttle_brake.update_throttle_brake_CAN_signals();
}

// wrapper for APPSs disagreement timer function
void APPSs_disagreement_timer_callback() {
  throttle_brake.set_is_APPSs_disagreement_implausibility_present_to_true();
}

// wrapper for brake implausible timer function
void brake_implausible_timer_callback() {
  throttle_brake.set_is_brake_shorted_or_opened_implausibility_present_to_true();
}

// call this function when the ready to drive switch is flipped
// currently, the switch is active low
void ready_to_drive_callback() {
  // update ready_to_drive_switch variable so we can see status of switch for testing
  if (digitalRead(static_cast<uint8_t>(Pins::READY_TO_DRIVE_SWITCH)) == LOW) {
    ready_to_drive_switch = Ready_To_Drive_State::Drive;
  } else {
    ready_to_drive_switch = Ready_To_Drive_State::Neutral;
  }

  // if the ready to drive switch is flipped and the brake is pressed, set ready_to_drive to drive
  if (digitalRead(static_cast<uint8_t>(Pins::READY_TO_DRIVE_SWITCH)) == LOW &&
      throttle_brake.is_brake_pressed()) {
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
    BMS_Command = BMSCommand::PrechargeAndCloseContactors;
  } else {
    tsactive_switch = TSActive::Inactive;
  }
}

void initialize_dash_switches() {
  // initialize Ready To Drive Switch -- use interrupts & pinMode
  pinMode(static_cast<uint8_t>(Pins::READY_TO_DRIVE_SWITCH), INPUT);
  attachInterrupt(digitalPinToInterrupt(static_cast<uint8_t>(Pins::READY_TO_DRIVE_SWITCH)),
                  ready_to_drive_callback, CHANGE);

  // initialize tsactive switch -- use interrupts & pinMode
  pinMode(static_cast<uint8_t>(Pins::TS_ACTIVE_PIN), INPUT);
  attachInterrupt(digitalPinToInterrupt(static_cast<uint8_t>(Pins::TS_ACTIVE_PIN)),
                  tsactive_callback, CHANGE);
}

// this function will be used to change the state of the vehicle based on the current state and the
// state of the switches
void change_state() {
  switch (Drive_State) {
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
}

// this function will be used to calculate torque based on LUTs and traction control when its time
void process_state() {
  throttle_brake.update_sensor_values();
  switch (Drive_State) {
    case State::OFF:
      if (tsactive_switch == TSActive::Active) {  // consider adding another state here for when
                                                  // TSactive switch is flipped, we can only get to
                                                  // this state on the rising edge of TS switch
        BMS_Command = BMSCommand::PrechargeAndCloseContactors;
      } else {
        BMS_Command = BMSCommand::Shutdown;
      }
      ready_to_drive = Ready_To_Drive_State::Neutral;
      // BMS_Command = BMSCommand::Shutdown;
      inverter.request_torque(0);
      break;
    case State::N:
      BMS_Command =
          BMSCommand::PrechargeAndCloseContactors;  // maybe make prechargeandclosecontactors or
                                                    // NoAction here
      inverter.request_torque(0);
      break;
    case State::DRIVE:
      // int32_t torque_req = calculate_torque(); // use this function to calculate torque based on
      // LUTs and traction control when its time
      int32_t torque_req;
      // if (throttle_brake.is_implausibility_present()) {
      //   torque_req = 0;
      // }
      // else {
      //   torque_req = static_cast<int32_t>(throttle_brake.get_throttle());
      // }
      torque_req = static_cast<int32_t>(throttle_brake.get_throttle());
      inverter.request_torque(torque_req);
      break;
  }
}

void print_fsm() {
  Serial.print("Drive State: ");
  switch (Drive_State) {
    case State::OFF:
      Serial.print("OFF");
      break;
    case State::N:
      Serial.print("N");
      break;
    case State::DRIVE:
      Serial.print("DRIVE");
      break;
  }
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
  Serial.print(" TS active switch: ");
  Serial.print(static_cast<int>(tsactive_switch));
  Serial.print(" is brake pressed: ");
  Serial.print(throttle_brake.is_brake_pressed());
  Serial.print(" Ready to Drive: ");
  Serial.print(static_cast<int>(ready_to_drive));
  Serial.print(" Ready to Drive Switch: ");
  Serial.print(static_cast<int>(ready_to_drive_switch));
  Serial.print(" Throttle: ");
  Serial.print(throttle_brake.get_throttle());
  throttle_brake.print_throttle_info();
  // inverter.print_inverter_info();

  // Serial.print("BMS msg: ");
  // Serial.println(static_cast<int>(BMS_State));

  Serial.println("");
}

void print_all() {
  print_fsm();
  inverter.print_inverter_info();
  throttle_brake.print_throttle_info();
  Serial.println("");
}

void tick_timers() {
  // Serial.println("tick timers");
  APPSs_disagree_timer.Tick(millis());
  brake_implausible_timer.Tick(millis());
  timers.Tick(millis());
}
// implausibility timer definitions
VirtualTimer APPSs_disagree_timer(
    100U, APPSs_disagreement_timer_callback,
    VirtualTimer::Type::
        kSingleUse);  // this timer needs to call
                      // Throttle_Brake::set_is_APPSs_disagreement_implausibility_present_to_true()
VirtualTimer brake_implausible_timer(
    100U, brake_implausible_timer_callback,
    VirtualTimer::Type::
        kSingleUse);  // this timer needs to call
                      // Throttle_Brake::set_is_brake_shorted_or_opened_implausibility_present_to_true()

// CAN signals -- get new addresses from DBC
// add rx: wheel speed
// add tx:
// APPS1, APPS2, front brake, rear brake, torque request will be handled in their respective .hpp
// files
CANSignal<BMSState, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    BMS_State{};  // says 1 bit in DBC .. im just using 8
// CANSignal<float, 40, 8, CANTemplateConvertFloat(0.5), CANTemplateConvertFloat(0), false>
// BMS_SOC{}; // says starts at bit 40 in DBC, also says size is 8 bits even tho its a float
CANSignal<BMSCommand, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    BMS_Command{};
CANSignal<State, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> Drive_State{};
CANRXMessage<1> BMS_Status{drive_bus, 0x175,
                           BMS_State};  //  BMS_SOC these addresses might be wrong, check DBC
CANTXMessage<1> ECU_BMS_Command_Message{drive_bus, 0x205, 1, 100, timers, BMS_Command};
CANTXMessage<1> ECU_Drive_Status{drive_bus, 0x206, 1, 100, timers, Drive_State};