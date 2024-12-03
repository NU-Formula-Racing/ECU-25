#pragma once
#include "esp_can.h"
#include "virtualTimer.h"
#include "throttle_brake_driver.hpp"
#include "inverter_driver.hpp"

// enum definitions
enum class BMSState 
{
  kShutdown = 0,
  kPrecharge = 1,
  kActive = 2,
  kCharging = 3,
  kFault = 4
};

enum class BMSCommand
{
  NoAction = 0,
  PrechargeAndCloseContactors = 1,
  Shutdown = 2
};

enum class TSActive
{
  Active,
  Inactive
};

enum class Ready_To_Drive_State
{
  Neutral = 1,
  Drive = 0
};

enum class Brake_State
{
  NotPressed,
  PressedInNeutral
};

enum class State
{
  OFF = 0,
  N = 1,
  DRIVE = 2
};

// instantiate CAN bus
ESPCAN drive_bus{};

// instantiate timer group
VirtualTimerGroup timers;

// instantiate throttle/brake timers
VirtualTimer APPSs_disagree_timer;
VirtualTimer brake_implausible_timer;

// instantiate throttle/brake
ThrottleBrake throttle_brake{drive_bus, APPSs_disagree_timer, brake_implausible_timer};

// instantiate inverter
Inverter inverter{drive_bus};

// function forward initializations
void init();
void change_state();
void process_state();
void change_brake_state();
void ready_to_drive_callback();
void tsactive_callback();
void initialize_dash_switches();
void send_inverter_CAN_wrapper();
void read_inverter_CAN_wrapper();
void send_throttle_brake_CAN_wrapper();

// global state variables
TSActive tsactive_switch;
Ready_To_Drive_State ready_to_drive;

// CAN signals -- get new addresses from DBC
// add rx: wheel speed
// add tx: 
// APPS1, APPS2, front brake, rear brake, torque request will be handled in their respective .hpp files
CANSignal<BMSState, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BMS_State{};
CANSignal<BMSCommand, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BMS_Command{};
CANSignal<float, 8, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(-40), false> batt_temp{};
CANSignal<State, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> current_state{};
CANRXMessage<2> BMS_message{drive_bus, 0x241, BMS_State, batt_temp};
CANTXMessage<1> BMS_command_message{drive_bus, 0x242, 8, 100, timers, BMS_Command};
CANTXMessage<1> Drive_status{drive_bus, 0x000, 8, 100, timers, current_state};