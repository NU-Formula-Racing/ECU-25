#include "CAN.hpp"

void CAN::initialize() {
  // instantiate CAN bus
  ESPCAN drive_bus{};
  // initialize CAN bus
  drive_bus.Initialize(ESPCAN::BaudRate::kBaud500K);
}

void CAN::RXupdate() {
  this->_RXdata.bmsstate = BMS_State{};
  this->_RXdata.fault = External_Kill_Fault{};

  this->_RXdata.FL_speed = FL_Speed{};
  this->_RXdata.FR_speed = FR_Speed{};
  this->_RXdata.BL_speed = BL_Speed{};
  this->_RXdata.BR_speed = BR_Speed{};

  this->_RXdata.max_discharge_curr = MAX_Discharge_Current{};
  this->_RXdata.max_regen_curr = MAX_Regen_Current{};
  this->_RXdata.battery_vol = Battery_Voltage{};
  this->_RXdata.battery_temp = Battery_Temperature{};
  this->_RXdata.battery_curr = Battery_Current{};
}

void CAN::TXupdata() {
  this->_TXdata.BMS_Command{} = command;  // tx
  this->_TXdata.Drive_State{} = state;    // tx
  this->_TXdata.
}