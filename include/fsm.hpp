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
static ESPCAN drive_bus{};

// instantiate timer group
static VirtualTimerGroup timers;

// instantiate throttle/brake timers
static VirtualTimer APPSs_disagree_timer;
static VirtualTimer brake_implausible_timer;

// instantiate throttle/brake
static ThrottleBrake throttle_brake{drive_bus, APPSs_disagree_timer, brake_implausible_timer};

// instantiate inverter
static Inverter inverter{drive_bus};

// function forward initializations
void fsm_init();
static void change_state();
static void process_state();
static void change_brake_state();
static void ready_to_drive_callback();
static void tsactive_callback();
static void initialize_dash_switches();
static void send_inverter_CAN_wrapper();
static void read_inverter_CAN_wrapper();
static void send_throttle_brake_CAN_wrapper();

// global state variables
static TSActive tsactive_switch;
static Ready_To_Drive_State ready_to_drive;

// CAN signals -- get new addresses from DBC
// add rx: wheel speed
// add tx: 
// APPS1, APPS2, front brake, rear brake, torque request will be handled in their respective .hpp files
extern CANSignal<BMSState, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BMS_State;
extern CANSignal<BMSCommand, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BMS_Command;
extern CANSignal<float, 8, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(-40), false> batt_temp;
extern CANSignal<State, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> current_state;
extern CANRXMessage<2> BMS_message;
extern CANTXMessage<1> BMS_command_message;
extern CANTXMessage<1> Drive_status;