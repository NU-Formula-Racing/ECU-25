#pragma once

#ifdef ESP32
#include "esp_can.h"
#endif
#include "can_interface.h"
#include "throttle_brake_driver.hpp"

class Inverter {
 public:
  Inverter(ICAN& can_interface_, VirtualTimerGroup& timer_group, ThrottleBrake& throttle_brake_)
      : can_interface(can_interface_), timers(timer_group), throttle_brake(throttle_brake_) {};
  void initialize();
  int32_t get_motor_rpm() const;
  int16_t get_IGBT_temp() const;
  int16_t get_motor_temp() const;
  void read_inverter_CAN();
  void send_inverter_CAN();
  void request_torque(int32_t torque_mA);
  void print_inverter_info();

 private:
  ICAN& can_interface;
  VirtualTimerGroup& timers;
  ThrottleBrake& throttle_brake;

  int32_t motor_rpm;
  int16_t IGBT_temp;
  int16_t motor_temp;
  int32_t requested_torque_throttle;
  int32_t requested_torque_brake;

  const uint16_t torque_limit = 32767;
  const uint16_t kTransmissionIDSetCurrent = 0x200;           // CAN msg address, get this from DBC
  const uint16_t kTransmissionIDSetCurrentBrake = 0x201;      // CAN msg address, get this from DBC
  const uint16_t kTransmissionIDInverterMotorStatus = 0x280;  // CAN msg address, get this from DBC
  const uint16_t kTransmissionIDInverterTempStatus = 0x281;   // CAN msg address, get this from DBC
  // CAN signals & msgs
  // tx: Set_Current, Set_Current_Brake
  CANSignal<int32_t, 0, 32, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true>
      Set_Current{};
  CANTXMessage<1> ECU_Set_Current{can_interface, kTransmissionIDSetCurrent, 4, 10, timers,
                                  Set_Current};
  CANSignal<int32_t, 0, 32, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true>
      Set_Current_Brake{};
  CANTXMessage<1> ECU_Set_Current_Brake{
      can_interface, kTransmissionIDSetCurrentBrake, 4, 10, timers, Set_Current_Brake};

  // rx: from inverter: motor temp, motor rpm, inverter/fet temp
  CANSignal<int16_t, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true> RPM{};
  CANRXMessage<1> Inverter_Motor_Status{can_interface, kTransmissionIDInverterMotorStatus,
                                        RPM /*, Motor_Current, DC_Voltage, DC_Current*/};
  CANSignal<int16_t, 0, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), true>
      IGBT_Temp{};
  CANSignal<int16_t, 16, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), true>
      Motor_Temp{};
  CANRXMessage<2> Inverter_Temp_Status{can_interface, kTransmissionIDInverterTempStatus, IGBT_Temp,
                                       Motor_Temp};
};