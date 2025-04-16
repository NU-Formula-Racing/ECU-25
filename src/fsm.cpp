#include "fsm.hpp"

#include <Arduino.h>

#include "esp_can.h"
#include "inverter_driver.hpp"
#include "pins.hpp"
#include "throttle_brake_driver.hpp"
#include "virtualTimer.h"

volatile int test_ts_active_switch_interrupt = 1;       // 1 OFF, 0 N
volatile int test_ready_to_drive_switch_interrupt = 1;  // 1 N, 0 D

// initialize state variables
TSActive tsactive_switch;
Ready_To_Drive_State ready_to_drive;
Ready_To_Drive_State ready_to_drive_switch;

// instantiate CAN bus
ESPCAN drive_bus{};

// instantiate timer group
VirtualTimerGroup timers;

// instantiate throttle/brake
ThrottleBrake throttle_brake{drive_bus, timers, APPSs_disagree_timer, brake_implausible_timer,
                             APPSs_invalid_timer};

// instantiate inverter
Inverter inverter{drive_bus, timers, throttle_brake};

// instantiate torque calculator
TorqueCalc torque_calc{};

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

// wrapper for APPSs invalid timer function
void APPSs_invalid_timer_callback() {
  throttle_brake.set_APPSs_invalid_implausibility_present_to_true();
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
    test_ready_to_drive_switch_interrupt = 0;
    ready_to_drive = Ready_To_Drive_State::Drive;
  } else {
    test_ready_to_drive_switch_interrupt = 1;
    ready_to_drive = Ready_To_Drive_State::Neutral;
  }
}

// call this function when the tsactive switch is flipped
// currently, the switch is active low
void tsactive_callback() {
  if (digitalRead((uint8_t)Pins::TS_ACTIVE_PIN) == LOW) {
    test_ts_active_switch_interrupt = 0;
    tsactive_switch = TSActive::Active;
    BMS_Command = BMSCommand::PrechargeAndCloseContactors;
  } else {
    test_ts_active_switch_interrupt = 1;
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

int32_t scale_torque_request(int16_t throttle) {
  // eventually:
  // if (throttle_brake.is_brake_pressed()) {
  //   torque_req = scale(LUT::get_brake_modifier(), 0, inverter.kRegenMax);
  // } else {
  //   torque_req = scale(LUT::get_throttle_modifier(), 0, inverter.kAccelMax);
  // }
  return static_cast<int32_t>(throttle_brake.get_throttle() / 4);
}

// this function will be used to change the state of the vehicle based on the current state and the
// state of the switches
void change_state() {
  switch (Drive_State) {
    case State::OFF:
      if (tsactive_switch == TSActive::Active && BMS_State == BMSState::kActive &&
          External_Kill_Fault == BMSFault::kNoExtFault) {
        Serial.println("transition from OFF->N");
        Drive_State = State::N;
      }
      break;

    case State::N:
      if (ready_to_drive == Ready_To_Drive_State::Drive &&
          External_Kill_Fault == BMSFault::kNoExtFault) {
        Serial.println("transition from N->DRIVE");
        Drive_State = State::DRIVE;
      }
      if (tsactive_switch == TSActive::Inactive || BMS_State == BMSState::kFault ||
          External_Kill_Fault == BMSFault::kExtFault) {
        Serial.println("transition from N->OFF");
        tsactive_switch = TSActive::Inactive;
        Drive_State = State::OFF;
      }
      break;

    case State::DRIVE:
      if (ready_to_drive == Ready_To_Drive_State::Neutral) {
        Serial.println("transition from DRIVE->N");
        Drive_State = State::N;
      }
      if (tsactive_switch == TSActive::Inactive || BMS_State == BMSState::kFault ||
          External_Kill_Fault == BMSFault::kExtFault) {
        Serial.println("transition from DRIVE->OFF");
        tsactive_switch = TSActive::Inactive;
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
      int32_t torque_req;
      if (throttle_brake.is_implausibility_present()) {
        torque_req = 0;
      } else {
        if (throttle_brake.is_brake_pressed()) {
          // calculate regen torque
        } else {
          torque_req = torque_calc.calculate_accel_torque(
              inverter.get_IGBT_temp(), Battery_Temperature, inverter.get_motor_rpm(),
              throttle_brake.get_throttle());
        }
      }
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
  Serial.print(" brake_pressed: ");
  Serial.print(throttle_brake.is_brake_pressed());
  Serial.print(" Ready to Drive: ");
  Serial.print(static_cast<int>(ready_to_drive));
  Serial.print(" Ready to Drive Switch: ");
  Serial.print(static_cast<int>(ready_to_drive_switch));
  Serial.print(" Thrtl: ");
  Serial.print(throttle_brake.get_throttle() / 4);
  throttle_brake.print_throttle_info();
  Serial.print(" test tsactive: ");
  Serial.print(test_ts_active_switch_interrupt);
  Serial.print(" test ready to drive: ");
  Serial.print(test_ready_to_drive_switch_interrupt);
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
  APPSs_invalid_timer.Tick(millis());
  timers.Tick(millis());
}
// implausibility timer definitions
VirtualTimer APPSs_disagree_timer(
    85U, APPSs_disagreement_timer_callback,
    VirtualTimer::Type::
        kSingleUse);  // this timer needs to call
                      // Throttle_Brake::set_is_APPSs_disagreement_implausibility_present_to_true()
VirtualTimer brake_implausible_timer(
    85U, brake_implausible_timer_callback,
    VirtualTimer::Type::
        kSingleUse);  // this timer needs to call
                      // Throttle_Brake::set_is_brake_shorted_or_opened_implausibility_present_to_true()

VirtualTimer APPSs_invalid_timer(
    85U, APPSs_invalid_timer_callback,
    VirtualTimer::Type::
        kSingleUse);  // this timer needs to call
                      // Throttle_Brake::set_APPSs_invalid_implausibility_present_to_true()

// CAN signals -- get new addresses from DBC
// add rx: wheel speed
// add tx:
// APPS1, APPS2, front brake, rear brake, torque request will be handled in their respective .hpp
// files
CANSignal<BMSState, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    BMS_State{};  // says 1 bit in DBC .. im just using 8
// CANSignal<float, 40, 8, CANTemplateConvertFloat(0.5), CANTemplateConvertFloat(0), false>
// BMS_SOC{}; // says starts at bit 40 in DBC, also says size is 8 bits even tho its a float
CANSignal<BMSFault, 6, 1, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    External_Kill_Fault{};
CANSignal<BMSCommand, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    BMS_Command{};
CANSignal<State, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> Drive_State{};
CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BL_Speed{};
CANSignal<float, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    BL_Displacement{};
CANSignal<float, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BL_Load{};
CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BR_Speed{};
CANSignal<float, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    BR_Displacement{};
CANSignal<float, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BR_Load{};
CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> FR_Speed{};
CANSignal<float, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    FR_Displacement{};
CANSignal<float, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> FR_Load{};
CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> FL_Speed;
CANSignal<float, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    FL_Displacement{};
CANSignal<float, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> FL_Load{};
CANSignal<float, 0, 12, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), false>
    MAX_Discharge_Current{};
CANSignal<float, 12, 12, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), false>
    MAX_Regen_Current{};
CANSignal<float, 24, 16, CANTemplateConvertFloat(0.01), CANTemplateConvertFloat(0), false>
    Battery_Voltage{};
CANSignal<float, 40, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(-40.0), false>
    Battery_Temperature{};
CANSignal<float, 48, 16, CANTemplateConvertFloat(0.01), CANTemplateConvertFloat(0), false>
    Battery_Current{};

CANRXMessage<3> Daq_Wheel_Bl{drive_bus, 0x24B, BL_Speed, BL_Displacement, BL_Load};
CANRXMessage<3> Daq_Wheel_BR{drive_bus, 0x24C, BR_Speed, BR_Displacement, BR_Load};
CANRXMessage<3> Daq_Wheel_FR{drive_bus, 0x249, FR_Speed, FR_Displacement, FR_Load};
CANRXMessage<3> Daq_Wheel_FL{drive_bus, 0x24A, FL_Speed, FL_Displacement, FL_Load};
CANRXMessage<5> BMS_SOE{drive_bus,         0x150,           MAX_Discharge_Current,
                        MAX_Regen_Current, Battery_Voltage, Battery_Temperature,
                        Battery_Current};
CANRXMessage<1> BMS_Status{drive_bus, 0x152, BMS_State};
CANRXMessage<1> BMS_Faults{drive_bus, 0x151, External_Kill_Fault};
CANTXMessage<1> ECU_BMS_Command_Message{drive_bus, 0x205, 1, 100, timers, BMS_Command};
CANTXMessage<1> ECU_Drive_Status{drive_bus, 0x206, 1, 100, timers, Drive_State};