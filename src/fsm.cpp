#include "fsm.hpp"

#include <Arduino.h>

#include "LUT.hpp"
#include "active_aero.hpp"
#include "esp_can.h"
#include "inverter_driver.hpp"
#include "pins.hpp"
#include "throttle_brake_driver.hpp"
#include "virtualTimer.h"

volatile int test_ts_active_switch_interrupt = 1;       // 1 OFF, 0 N
volatile int test_ready_to_drive_switch_interrupt = 1;  // 1 N, 0 D,

// initialize state variables
TSActive tsactive_switch;
Ready_To_Drive_State ready_to_drive;
Ready_To_Drive_State ready_to_drive_switch;

// instantiate CAN bus
ESPCAN drive_bus{100U, GPIO_NUM_5, GPIO_NUM_4};

// instantiate timer group
VirtualTimerGroup timers;

// instantiate throttle/brake
ThrottleBrake throttle_brake{drive_bus, timers, APPSs_disagree_timer, brake_implausible_timer,
                             APPSs_invalid_timer};

// instantiate inverter
Inverter inverter{drive_bus, timers, throttle_brake};

ActiveAero active_aero{drive_bus, timers};

Lookup lookup{drive_bus, timers};

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

  timers.AddTimer(10, update);

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

void update() {
  process_state();
  change_state();

  inverter.read_inverter_CAN();
  inverter.send_inverter_CAN();

  throttle_brake.update_sensor_values();
  throttle_brake.check_for_implausibilities();
  throttle_brake.update_throttle_brake_CAN_signals();

  active_aero.update_active_aero(inverter.get_set_current(),
                                 static_cast<float>(Lookup::TorqueReqLimit::kAccelMax),
                                 throttle_brake.is_brake_pressed());

  Pump_Duty_Cycle = lookup.calculate_pump_duty_cycle(inverter.get_motor_temp(),
                                                     inverter.get_IGBT_temp(), Battery_Temperature);
  Fan_Duty_Cycle = lookup.calculate_fan_duty_cycle(Before_Motor_Temperature);

  // lookup.updateCANLUTs();
  lookup.update_status_CAN();
  drive_bus.Tick();
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
  if (digitalRead((uint8_t)Pins::TS_ACTIVE_PIN) == LOW &&
      External_Kill_Fault == BMSFault::kNoExtFault) {
    test_ts_active_switch_interrupt = 0;
    tsactive_switch = TSActive::Active;
    BMS_Command = BMSCommand::PrechargeAndCloseContactors;
  } else {
    test_ts_active_switch_interrupt = 1;
    tsactive_switch = TSActive::Inactive;
    BMS_Command = BMSCommand::Shutdown;
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
        BMS_Command = BMSCommand::Shutdown;
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
        BMS_Command = BMSCommand::Shutdown;
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
      if (tsactive_switch == TSActive::Active) {
        BMS_Command = BMSCommand::PrechargeAndCloseContactors;
      } else {
        BMS_Command = BMSCommand::Shutdown;
      }
      ready_to_drive = Ready_To_Drive_State::Neutral;
      // BMS_Command = BMSCommand::Shutdown;
      inverter.request_torque({0, 0});
      break;
    case State::N:
      BMS_Command =
          BMSCommand::PrechargeAndCloseContactors;  // maybe make prechargeandclosecontactors or
                                                    // NoAction here
      inverter.request_torque({0, 0});
      break;
    case State::DRIVE:
      std::pair<int32_t, int32_t> torque_reqs;
      if (throttle_brake.is_implausibility_present()) {
        torque_reqs = {0, 0};
      } else {
        std::pair<float, float> torque_mods = lookup.get_torque_mods(
            throttle_brake.get_throttle(), static_cast<int16_t>(Bounds::SENSOR_SCALED_MAX),
            inverter.get_motor_rpm(), throttle_brake.is_brake_pressed());

        float temp_mod = lookup.calculate_temp_mod(inverter.get_IGBT_temp(), Battery_Temperature,
                                                   inverter.get_motor_temp());

        torque_reqs = lookup.calculate_torque_reqs(inverter.get_motor_rpm(), temp_mod, torque_mods);
      }
      inverter.request_torque(torque_reqs);
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
  // Serial.print(" Thrtl: ");
  // Serial.print(throttle_brake.get_throttle() / 4);
  throttle_brake.print_throttle_info();

  // inverter.print_inverter_info();
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
CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BR_Speed{};
CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> FR_Speed{};
CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> FL_Speed{};

CANSignal<float, 40, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(-40.0), false>
    Battery_Temperature{};

CANSignal<uint8_t, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    Pump_Duty_Cycle{};
CANSignal<uint8_t, 8, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    Fan_Duty_Cycle{};

CANSignal<float, 0, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), false>
    Before_Motor_Temperature{};

CANRXMessage<1> Daq_Wheel_Bl{drive_bus, 0x24B, BL_Speed};
CANRXMessage<1> Daq_Wheel_BR{drive_bus, 0x24C, BR_Speed};
CANRXMessage<1> Daq_Wheel_FR{drive_bus, 0x249, FR_Speed};
CANRXMessage<1> Daq_Wheel_FL{drive_bus, 0x24A, FL_Speed};
CANRXMessage<1> BMS_SOE{
    drive_bus,
    0x150,
    Battery_Temperature,
};
CANRXMessage<1> BMS_Status{drive_bus, 0x152, BMS_State};
CANRXMessage<1> BMS_Faults{drive_bus, 0x151, External_Kill_Fault};
CANRXMessage<1> DAQ_Coolant_Temps{drive_bus, 0x135, Before_Motor_Temperature};

CANTXMessage<1> ECU_BMS_Command_Message{drive_bus, 0x205, 1, 100, timers, BMS_Command};
CANTXMessage<1> ECU_Drive_Status{drive_bus, 0x206, 1, 100, timers, Drive_State};
CANTXMessage<2> ECU_Pump_Fan_Command{drive_bus,       0x209,         2, 100, timers,
                                     Pump_Duty_Cycle, Fan_Duty_Cycle};