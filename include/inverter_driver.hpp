#pragma once

#ifdef ESP32
#include "esp_can.h"
#endif
#include "can_interface.h"


class Inverter {
    public:
        Inverter(ICAN &can_interface_) : can_interface(can_interface_){};
        void initialize();
        int32_t get_motor_rpm();
        int16_t get_IGBT_temp();
        int16_t get_motor_temp();
        void read_inverter_CAN();
        void send_inverter_CAN();
        void request_torque(int32_t torque_mA); // how do we want to do the torque request function -- send torque in mA
        void print_inverter_info();
         
    private:
        ICAN &can_interface;
        int32_t motor_rpm;
        int16_t IGBT_temp;
        int16_t motor_temp;
        int32_t requested_torque_throttle;
        int32_t requested_torque_brake;
        
        const uint16_t torque_limit = 32767;
        const uint16_t kTransmissionIDSetCurrent = 0x200; // CAN msg address, get this from DBC 
        const uint16_t kTransmissionIDSetCurrentBrake = 0x201; // CAN msg address, get this from DBC 
        const uint16_t kTransmissionIDInverterMotorStatus = 0x280; // CAN msg address, get this from DBC 
        const uint16_t kTransmissionIDInverterTempStatus = 0x281; // CAN msg address, get this from DBC 
        // CAN signals & msgs 
        // tx: Set_Current, Set_Current_Brake
        CANSignal<int32_t, 0, 32, CANTemplateConvertFloat(0.001), CANTemplateConvertFloat(0), true> Set_Current{};
        CANTXMessage<1> ECU_Set_Current{can_interface, kTransmissionIDSetCurrent, 8, 100, Set_Current};
        CANSignal<int32_t, 0, 32, CANTemplateConvertFloat(0.001), CANTemplateConvertFloat(0), true> Set_Current_Brake{};
        CANTXMessage<1> ECU_Set_Current_Brake{can_interface, kTransmissionIDSetCurrentBrake, 8, 100, Set_Current_Brake};

        // rx: from inverter: motor temp, motor rpm, inverter/fet temp
        CANSignal<int16_t, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true> RPM{};
        // CANSignal<int16_t, 16, 32, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), true> Motor_Current{}; // dont need this for LUTs
        // CANSignal<int16_t, 32, 48, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), true> DC_Voltage{}; // dont need this for LUTs
        // CANSignal<int16_t>, 48, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), true> DC_Current{}; // dont need this for LUTs
        CANRXMessage<1> Inverter_Motor_Status{
            can_interface, kTransmissionIDInverterMotorStatus, 
            RPM /*, Motor_Current, DC_Voltage, DC_Current*/};
        CANSignal<int16_t, 0, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), true> IGBT_Temp{};
        CANSignal<int16_t, 16, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), true> Motor_Temp{};
        CANRXMessage<2> Inverter_Temp_Status{
            can_interface, kTransmissionIDInverterTempStatus,
            IGBT_Temp, Motor_Temp};
};