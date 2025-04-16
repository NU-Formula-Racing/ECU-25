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

  this->_RXdata.front_RXdata[0][0] = FL_Speed{};  // ex. update front speed-left
  this->_RXdata.front_RXdata[1][0] = FL_Load{};
  this->_RXdata.front_RXdata[2][0] = FL_Displacement{};

  this->_RXdata.front_RXdata[0][1] = FR_Speed{};
  this->_RXdata.front_RXdata[1][1] = FR_Load{};
  this->_RXdata.front_RXdata[2][1] = FR_Displacement{};

  this->_RXdata.back_RXdata[0][0] = BL_Speed{};
  this->_RXdata.back_RXdata[1][0] = BL_Load{};
  this->_RXdata.back_RXdata[2][0] = BL_Displacement{};

  this->_RXdata.back_RXdata[0][1] = BR_Speed{};
  this->_RXdata.back_RXdata[1][1] = BR_Load{};
  this->_RXdata.back_RXdata[2][1] = BR_Displacement{};

  this->_RXdata.max_discharge_curr = MAX_Discharge_Current{};
  this->_RXdata.max_regen_curr = MAX_Regen_Current{};
  this->_RXdata.battery_vol = Battery_Voltage{};
  this->_RXdata.battery_temp = Battery_Temperature{};
  this->_RXdata.battery_curr = Battery_Current{};

}

void CAN::TXupdata(){
  this->_TXdata.BMS_Command{} = command;  // tx
  this->_TXdata.Drive_State{} = state;    // tx
}