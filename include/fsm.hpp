#pragma once
#include <Arduino.h>

#include "LUT.hpp"
#include "esp_can.h"
#include "inverter_driver.hpp"
#include "throttle_brake_driver.hpp"
#include "virtualTimer.h"

// enum definitions
enum class BMSState { kShutdown = 0, kPrecharge = 1, kActive = 2, kCharging = 3, kFault = 4 };

enum class BMSFault { kNoExtFault = 0, kExtFault = 1 };

enum class BMSCommand { PrechargeAndCloseContactors = 0, Shutdown = 1 };

enum class TSActive { Active = 0, Inactive = 1 };

enum class Ready_To_Drive_State { Neutral = 1, Drive = 0 };

enum class Brake_State { NotPressed = 0, PressedInNeutral = 1 };

enum class State { OFF = 0, N = 1, DRIVE = 2 };

// instantiate CAN bus
extern ESPCAN drive_bus;

// instantiate timer group
extern VirtualTimerGroup timers;

// instantiate throttle/brake timers
extern VirtualTimer
    APPSs_disagree_timer;  // this timer needs to call
                           // Throttle_Brake::set_is_APPSs_disagreement_implausibility_present_to_true()
extern VirtualTimer
    brake_implausible_timer;  // this timer needs to call
                              // Throttle_Brake::set_is_brake_shorted_or_opened_implausibility_present_to_true()

extern VirtualTimer
    APPSs_invalid_timer;  // this timer needs to call
                          // Throttle_Brake::set_APPSs_invalid_implausibility_present_to_true()

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
void update_inverter();
void active_aero_wrapper();
void send_throttle_brake_CAN_wrapper();
void APPSs_disagreement_timer_callback();
void brake_implausible_timer_callback();
void APPSs_invalid_timer_callback();
void refresh_throttle_brake();
int32_t scale_torque_request(int16_t throttle);
void tick_CAN();
void print_fsm();
void print_all();
void tick_timers();

// global state variables
extern TSActive tsactive_switch;  // physical status of the tsactive dashboard switch
extern Ready_To_Drive_State
    ready_to_drive_switch;  // physical status of the ready to drive dashboard switch
extern Ready_To_Drive_State ready_to_drive;  // goes to drive when the when the brake is held while
                                             // the ready_to_drive switch is flipped

// CAN signals
extern CANSignal<BMSState, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    BMS_State;  // says 1 bit in DBC .. im just using 8
extern CANSignal<float, 40, 8, CANTemplateConvertFloat(0.5), CANTemplateConvertFloat(0), false>
    BMS_SOC;  // says starts at bit 40 in DBC, also says size is 8 bits even tho its a float
extern CANSignal<BMSFault, 6, 1, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    External_Kill_Fault;

extern CANSignal<BMSCommand, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    BMS_Command;

extern CANSignal<State, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    Drive_State;

extern CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    BL_Speed;
extern CANSignal<float, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    BL_Displacement;
extern CANSignal<float, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    BL_Load;

extern CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    BR_Speed;
extern CANSignal<float, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    BR_Displacement;
extern CANSignal<float, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    BR_Load;

extern CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    FR_Speed;
extern CANSignal<float, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    FR_Displacement;
extern CANSignal<float, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    FR_Load;

extern CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    FL_Speed;
extern CANSignal<float, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    FL_Displacement;
extern CANSignal<float, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    FL_Load;

extern CANSignal<float, 0, 12, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), false>
    MAX_Discharge_Current;
extern CANSignal<float, 12, 12, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), false>
    MAX_Regen_Current;
extern CANSignal<float, 24, 16, CANTemplateConvertFloat(0.01), CANTemplateConvertFloat(0), false>
    Battery_Voltage;
extern CANSignal<float, 40, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(-40.0), false>
    Battery_Temperature;
extern CANSignal<float, 48, 16, CANTemplateConvertFloat(0.01), CANTemplateConvertFloat(0), false>
    Battery_Current;

extern CANSignal<LUT::LUTChoice, 0, 1, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0),
                 false>
    LUT_Choice;

extern CANSignal<uint8_t, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    Pump_Duty_Cycle;
extern CANSignal<uint8_t, 8, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
    Fan_Duty_Cycle;

extern CANRXMessage<1> ECU_TEST_Throttle_Map_Choice;
extern CANRXMessage<5> BMS_SOE;
extern CANRXMessage<3> DAQ_Wheel_BL;
extern CANRXMessage<3> DAQ_Wheel_BR;
extern CANRXMessage<3> DAQ_Wheel_FR;
extern CANRXMessage<3> DAQ_Wheel_FL;
extern CANRXMessage<1> BMS_Status;
extern CANRXMessage<1> BMS_Faults;
extern CANTXMessage<1> ECU_BMS_Command_Message;
extern CANTXMessage<1> ECU_Drive_Status;
extern CANTXMessage<2> ECU_Pump_Fan_Command;