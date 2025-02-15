#pragma once
#include <Arduino.h>

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
  Active = 0,
  Inactive = 1
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
extern ESPCAN drive_bus;

// instantiate timer group
extern VirtualTimerGroup timers;

// instantiate throttle/brake timers
extern VirtualTimer APPSs_disagree_timer; // this timer needs to call Throttle_Brake::set_is_APPSs_disagreement_implausibility_present_to_true()
extern VirtualTimer brake_implausible_timer; // this timer needs to call Throttle_Brake::set_is_brake_shorted_or_opened_implausibility_present_to_true()

// instantiate throttle/brake
extern ThrottleBrake throttle_brake;

// instantiate inverter
extern Inverter inverter;

// function forward initializations
void fsm_init();
void change_state();
void process_state();
void change_brake_state();
void ready_to_drive_callback();
void tsactive_callback();
void initialize_dash_switches();
void send_inverter_CAN_wrapper();
void read_inverter_CAN_wrapper();
void send_throttle_brake_CAN_wrapper();
void APPSs_disagreement_timer_callback();
void brake_implausible_timer_callback();
void refresh_throttle_brake();
void tick_CAN();
void print_fsm();
void print_all();
void tick_timers();

// global state variables
extern TSActive tsactive_switch; // physical status of the tsactive dashboard switch
extern Ready_To_Drive_State ready_to_drive_switch; // physical status of the ready to drive dashboard switch
extern Ready_To_Drive_State ready_to_drive; // goes to drive when the when the brake is held while the ready_to_drive switch is flipped

// CAN signals
extern CANSignal<BMSState, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BMS_State;
extern CANSignal<BMSCommand, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BMS_Command;
extern CANSignal<float, 8, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(-40), false> batt_temp;
extern CANSignal<State, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> Drive_State;
extern CANRXMessage<2> BMS_Message;
extern CANTXMessage<1> BMS_Command_Message;
extern CANTXMessage<1> Drive_Status;