#pragma once

#include "esp_can.h"
#include "implausibility.hpp"
#include "virtualTimer.h"

struct CANRXData {
  BMSState bms_state;
  BMSFault bms_fault;
  float battery_temp;

  int16_t igbt_temp;
  int16_t motor_temp;

  float FL_speed;
  float FR_speed;
  float BL_speed;
  float BR_speed;
};

// TODO: make structs for groups of these if they come from a class together
// do this after all the other classes are made
struct CANTXData {
  BMSCommand bms_command;

  State drive_state;

  int32_t set_current;
  int32_t set_current_brake;

  ImplausibilitiesData implausibilities;

  int16_t apps1_throttle;
  int16_t apps2_throttle;
  int16_t front_brake_pressure;
  int16_t rear_brake_pressure;
  bool brake_pressed;
};

// TODO: finish populating this enum with all the addresses
// TODO: fix typos
// TODO: check DBC for addresses
enum class CANAddress {
  kBMSStatusID = 0x152,
  kBMSFaultsID = 0x151,
  kBMSSOEID = 0x150,

  kInversterTempStatusID = kTransmissionIDInverterTempStatus,
  kBackLeftID = 0x24B,
  kBackRightID = 0x24C,
  kFrontLeftID = 0x24A,
  KFrontRigthID = 0x249

};

class CAN {
 public:
  void initialize();
  void RXupdate();
  void TXupdate();

 private:
  CANRXData _RXdata;
  CANTXData _TXdata;

  ESPCAN _drive_bus;
  VirtualTimerGroup CAN_timers{};

  //// RX
  // BMS (BMS State, External (Shutdown Circuit) Fault, Battery Temp)
  CANSignal<BMSState, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      BMS_State{};
  CANSignal<BMSFault, 6, 1, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      External_Kill_Fault{};
  CANSignal<float, 40, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(-40.0), false>
      Battery_Temperature{};
  CANRXMessage<1> BMS_Status{_drive_bus, 0x152, BMS_State};
  CANRXMessage<1> BMS_Faults{_drive_bus, 0x151, External_Kill_Fault};
  CANRXMessage<1> BMS_SOE{_drive_bus, 0x150, Battery_Temperature};

  // Inverter (IGBT Temp, Motor Temp)
  CANSignal<int16_t, 0, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), true>
      IGBT_Temp{};
  CANSignal<int16_t, 16, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), true>
      Motor_Temp{};
  CANRXMessage<2> Inverter_Temp_Status{_drive_bus, kTransmissionIDInverterTempStatus, IGBT_Temp,
                                       Motor_Temp};

  // Wheel Brokers (Wheel Speed)
  CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BL_Speed{};
  CANRXMessage<1> Daq_Wheel_Bl{_drive_bus, 0x24B, BL_Speed};

  CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BR_Speed{};
  CANRXMessage<1> Daq_Wheel_BR{_drive_bus, 0x24C, BR_Speed};

  CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> FR_Speed{};
  CANRXMessage<1> Daq_Wheel_FR{_drive_bus, 0x249, FR_Speed};

  CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> FL_Speed;
  CANRXMessage<1> Daq_Wheel_FL{_drive_bus, 0x24A, FL_Speed};

  //// TX
  // BMS Command
  CANSignal<BMSCommand, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      BMS_Command{};
  CANTXMessage<1> ECU_BMS_Command_Message{_drive_bus, 0x205, 1, 100, CAN_timers, BMS_Command};

  // Drive State
  CANSignal<State, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      Drive_State{};
  CANTXMessage<1> ECU_Drive_Status{_drive_bus, 0x206, 1, 100, CAN_timers, Drive_State};

  // Set Current & Set Current Brake
  CANSignal<int32_t, 0, 32, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true>
      Set_Current{};
  CANTXMessage<1> ECU_Set_Current{_drive_bus, kTransmissionIDSetCurrent, 4, 10, CAN_timers,
                                  Set_Current};
  CANSignal<int32_t, 0, 32, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true>
      Set_Current_Brake{};
  CANTXMessage<1> ECU_Set_Current_Brake{
      _drive_bus, kTransmissionIDSetCurrentBrake, 4, 10, CAN_timers, Set_Current_Brake};

  // Implausibilities
  CANSignal<bool, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      CAN_Implausibility_Present{};
  CANSignal<bool, 8, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      CAN_APPSs_Disagreement_Imp{};
  CANSignal<bool, 16, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      CAN_BPPC_Imp{};
  CANSignal<bool, 24, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      CAN_Brake_Invalid_Imp{};
  CANSignal<bool, 32, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      CAN_APPSs_Invalid_Imp{};
  CANTXMessage<5> ECU_Implausibility{_drive_bus,
                                     kTransmissionIDImplausibility,
                                     5,
                                     100,
                                     CAN_timers,
                                     CAN_Implausibility_Present,
                                     CAN_APPSs_Disagreement_Imp,
                                     CAN_BPPC_Imp,
                                     CAN_Brake_Invalid_Imp,
                                     CAN_APPSs_Invalid_Imp};

  // Sensor Data (APPS, Brakes)
  CANSignal<int16_t, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true>
      APPS1_Throttle{};
  CANSignal<int16_t, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true>
      APPS2_Throttle{};
  CANTXMessage<2> ECU_Throttle{_drive_bus,     kTransmissionIDThrottle, 4, 100, CAN_timers,
                               APPS1_Throttle, APPS2_Throttle};
  CANSignal<int16_t, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true>
      Front_Brake_Pressure{};
  CANSignal<int16_t, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true>
      Rear_Brake_Pressure{};
  CANSignal<bool, 32, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      Brake_Pressed{};
  CANTXMessage<3> ECU_Brake{
      _drive_bus,          kTransmissionIDBrake, 5, 100, CAN_timers, Front_Brake_Pressure,
      Rear_Brake_Pressure, Brake_Pressed};
}