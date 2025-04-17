#include "esp_can.h"
#include "implausibilities.hpp"
#include "virtualTimer.h"

struct CANRXData {
  BMSState bmsstate;
  BMSFault fault;

  float FL_speed;
  float FR_speed;
  float BL_speed;
  float BR_speed;

  float max_discharge_curr;
  float max_regen_curr;
  float battery_vol;
  float battery_temp;
  float battery_curr;
};

struct CANTXData {
  BMSCommand command;
  State state;
  ImplausibilitiesPresent implausibilities;
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
  // BMS
  CANSignal<BMSState, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      BMS_State{};
  // CANSignal<float, 40, 8, CANTemplateConvertFloat(0.5), CANTemplateConvertFloat(0), false>
  // BMS_SOC{}; // says starts at bit 40 in DBC, also says size is 8 bits even tho its a float
  CANSignal<BMSFault, 6, 1, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      External_Kill_Fault{};
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
  CANRXMessage<1> BMS_Status{_drive_bus, 0x152, BMS_State};
  CANRXMessage<1> BMS_Faults{_drive_bus, 0x151, External_Kill_Fault};
  CANRXMessage<5> BMS_SOE{_drive_bus,        0x150,           MAX_Discharge_Current,
                          MAX_Regen_Current, Battery_Voltage, Battery_Temperature,
                          Battery_Current};

  // Inverter (IGBT Temp, Motor Temp)
  CANSignal<int16_t, 0, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), true>
      IGBT_Temp{};
  CANSignal<int16_t, 16, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), true>
      Motor_Temp{};
  CANRXMessage<2> Inverter_Temp_Status{can_interface, kTransmissionIDInverterTempStatus, IGBT_Temp,
                                       Motor_Temp};

  // DAQ (wheel speed)

  CANSignal<BMSCommand, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      BMS_Command{};
  CANTXMessage<1> ECU_BMS_Command_Message{_drive_bus, 0x205, 1, 100, timers, BMS_Command};

  CANSignal<State, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      Drive_State{};
  CANTXMessage<1> ECU_Drive_Status{_drive_bus, 0x206, 1, 100, timers, Drive_State};

  CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BL_Speed{};
  CANSignal<float, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      BL_Displacement{};
  CANSignal<float, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BL_Load{};
  CANRXMessage<3> Daq_Wheel_Bl{_drive_bus, 0x24B, BL_Speed, BL_Displacement, BL_Load};

  CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BR_Speed{};
  CANSignal<float, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      BR_Displacement{};
  CANSignal<float, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BR_Load{};
  CANRXMessage<3> Daq_Wheel_BR{_drive_bus, 0x24C, BR_Speed, BR_Displacement, BR_Load};

  CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> FR_Speed{};
  CANSignal<float, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      FR_Displacement{};
  CANSignal<float, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> FR_Load{};
  CANRXMessage<3> Daq_Wheel_FR{_drive_bus, 0x249, FR_Speed, FR_Displacement, FR_Load};

  CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> FL_Speed;
  CANSignal<float, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      FL_Displacement{};
  CANSignal<float, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> FL_Load{};
  CANRXMessage<3> Daq_Wheel_FL{_drive_bus, 0x24A, FL_Speed, FL_Displacement, FL_Load};

  CANSignal<int16_t, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true>
      APPS1_Throttle{};
  CANSignal<int16_t, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true>
      APPS2_Throttle{};
  CANTXMessage<2> ECU_Throttle{can_interface,  kTransmissionIDThrottle, 4, 100, timers,
                               APPS1_Throttle, APPS2_Throttle};
  CANSignal<int16_t, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true>
      Front_Brake_Pressure{};
  CANSignal<int16_t, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true>
      Rear_Brake_Pressure{};
  CANSignal<bool, 32, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      Brake_Pressed{};
  CANTXMessage<3> ECU_Brake{
      can_interface,       kTransmissionIDBrake, 5, 100, timers, Front_Brake_Pressure,
      Rear_Brake_Pressure, Brake_Pressed};
  CANSignal<bool, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      CAN_Implausibility_Present{};
  CANSignal<bool, 8, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      CAN_APPSs_Disagreement_Imp{};
  CANSignal<bool, 16, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      CAN_BPPC_Imp{};
  CANSignal<bool, 24, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      CAN_Brake_invalid_Imp{};
  CANSignal<bool, 32, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      CAN_APPSs_Invalid_Imp{};
  CANTXMessage<5> ECU_Implausibility{can_interface,
                                     kTransmissionIDImplausibility,
                                     5,
                                     100,
                                     timers,
                                     CAN_Implausibility_Present,
                                     CAN_APPSs_Disagreement_Imp,
                                     CAN_BPPC_Imp,
                                     CAN_Brake_invalid_Imp,
                                     CAN_APPSs_Invalid_Imp};

  CANSignal<int32_t, 0, 32, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true>
      Set_Current{};
  CANTXMessage<1> ECU_Set_Current{can_interface, kTransmissionIDSetCurrent, 4, 10, timers,
                                  Set_Current};
  CANSignal<int32_t, 0, 32, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true>
      Set_Current_Brake{};
  CANTXMessage<1> ECU_Set_Current_Brake{
      can_interface, kTransmissionIDSetCurrentBrake, 4, 10, timers, Set_Current_Brake};
}