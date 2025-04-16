

struct CANRXData {
  BMSState bmsstate;
  BMSFault fault;

  // Wheel Data:
  // Speed (LR)
  // Displacement (LR)
  // Load (LR)
  float front_data[3][2];
  float back_data[3][2];

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
  VirtualTimerGroup CAN_timers;

  // CAN signals -- get new addresses from DBC
  // add rx: wheel speed
  // add tx:
  // APPS1, APPS2, front brake, rear brake, torque request will be handled in their respective .hpp
  // files
  CANSignal<BMSState, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      BMS_State{};  // says 1 bit in DBC .. im just using 8
  // CANSignal<float, 40, 8, CANTemplateConvertFloat(0.5), CANTemplateConvertFloat(0), false>
  // BMS_SOC{}; // says starts at bit 40 in DBC, also says size is 8 bits even tho its a float
  CANRXMessage<1> BMS_Status{_drive_bus, 0x152, BMS_State};

  CANSignal<BMSFault, 6, 1, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      External_Kill_Fault{};
  CANRXMessage<1> BMS_Faults{_drive_bus, 0x151, External_Kill_Fault};

  CANSignal<BMSCommand, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      BMS_Command{};
  CANTXMessage<1> ECU_BMS_Command_Message{_drive_bus, 0x205, 1, 100, timers, BMS_Command};

  CANSignal<State, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      Drive_State{};
  CANTXMessage<1> ECU_Drive_Status{_drive_bus, 0x206, 1, 100, timers, Drive_State};

  CANRXMessage<5> BMS_SOE{_drive_bus,        0x150,           MAX_Discharge_Current,
                          MAX_Regen_Current, Battery_Voltage, Battery_Temperature,
                          Battery_Current};

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
}