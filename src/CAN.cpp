#include "CAN.hpp"

void CAN::initialize() {
  // instantiate CAN bus
  ESPCAN drive_bus{};
  // initialize CAN bus
  drive_bus.Initialize(ESPCAN::BaudRate::kBaud500K);
}

void CAN::RXupdate() {
  _RXdata.bms_state = BMS_State;
  _RXdata.bms_fault = External_Kill_Fault;
  _RXdata.battery_temp = Battery_Temperature;

  _RXdata.igbt_temp = IGBT_Temp;
  _RXdata.motor_temp = Motor_Temp;

  _RXdata.FL_speed = FL_Speed;
  _RXdata.FR_speed = FR_Speed;
  _RXdata.BL_speed = BL_Speed;
  _RXdata.BR_speed = BR_Speed;
}

void CAN::TXupdate() {
  BMS_Command = _TXdata.bms_command;

  Drive_State = _TXdata.drive_state;

  Set_Current = _TXdata.set_current;
  Set_Current_Brake = _TXdata.set_current_brake;

  CAN_Implausibility_Present = _TXdata.implausibilities.any_implausibility_present;
  CAN_APPSs_Disagreement_Imp = _TXdata.implausibilities.APPSs_disagreement_implausibility_present;
  CAN_BPPC_Imp = _TXdata.implausibilities.BPPC_implausibility_present;
  CAN_Brake_Invalid_Imp = _TXdata.implausibilities.brake_shorted_or_opened_implausibility_present;
  CAN_APPSs_Invalid_Imp = _TXdata.implausibilities.APPSs_invalid_implausibility_present;

  APPS1_Throttle = _TXdata.apps1_throttle;
  APPS2_Throttle = _TXdata.apps2_throttle;
  Front_Brake_Pressure = _TXdata.front_brake_pressure;
  Rear_Brake_Pressure = _TXdata.rear_brake_pressure;
  Brake_Pressed = _TXdata.brake_pressed;
}