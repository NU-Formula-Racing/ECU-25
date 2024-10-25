#pragma once

#include "esp_can.h"
#include "can_interface.h"


class Inverter {
    public:
        Inverter(ICAN &can_interface_) : can_interface(can_interface_){};
        void initialize();
        void get_motor_rpm();
        void get_IGBT_temp();
        void get_motor_temp();
        void read_inverter_CAN();
        void send_inverter_CAN();
        void request_torque(int32_t torque_mA); // how do we want to do the torque request function -- send torque in mA
        
    private:
        ICAN &can_interface;
        int32_t motor_rpm;
        int16_t IGBT_temp;
        int16_t motor_temp;
        int32_t requested_torque;
        
        const uint16_t torque_limit = 32767;
        const uint16_t set_current_transmissionID = 0x100; // CAN msg address, get this from DBC (0x1xx)
        const uint16_t set_current_brake_TransmissionID = 0x200; // CAN msg address, get this from DBC (0x2xx)
        const uint16_t inverter_status_1_transmissionID = 0x900; // CAN msg address, get this from DBC (0x9xx)
        const uint16_t inverter_status_2_transmissionID = 0x1600; // CAN msg address, get this from DBC (0x16xx)
        // CAN signals & msgs 
        // tx: set_current, set_current_brake
        CANSignal<int32_t, 0, 32, CANTemplateConvertFloat(1000), CANTemplateConvertFloat(0), false> set_current{};
        CANTXMessage<1> Set_Current{can_interface, set_current_transmissionID, 8, 100, set_current};
        CANSignal<int32_t, 0, 32, CANTemplateConvertFloat(1000), CANTemplateConvertFloat(0), false> set_current_brake{};
        CANTXMessage<1> Set_Current_Brake{can_interface, set_current_brake_TransmissionID, 8, 100, set_current_brake};

        // rx: from inverter: motor temp, motor rpm, inverter/fet temp
        CANSignal<int32_t, 0, 32, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> ERPM{};
        CANSignal<int16_t, 32, 16, CANTemplateConvertFloat(10), CANTemplateConvertFloat(0), false> Motor_Current{}; // dont need this for LUTs
        CANSignal<uint16_t, 48, 16, CANTemplateConvertFloat(1000), CANTemplateConvertFloat(0), false> Motor_Duty_Cycle{}; // dont need this for LUTs
        CANRXMessage<3> Inverter_Status_1{
            can_interface, inverter_status_1_transmissionID, 
            ERPM, Motor_Current, Motor_Duty_Cycle};
        CANSignal<int16_t, 0, 16, CANTemplateConvertFloat(10), CANTemplateConvertFloat(0), false> Temp_FET{};
        CANSignal<int16_t, 16, 16, CANTemplateConvertFloat(10), CANTemplateConvertFloat(0), false> Temp_Motor{};
        CANSignal<int16_t, 32, 16, CANTemplateConvertFloat(10), CANTemplateConvertFloat(0), false> Current_In{}; // dont need this for LUTs
        CANSignal<int16_t, 48, 16, CANTemplateConvertFloat(50), CANTemplateConvertFloat(0), false> PID_Pos{}; // dont need this for LUTs
        CANRXMessage<4> Inverter_Status_4{
            can_interface, inverter_status_2_transmissionID, 
            Temp_FET, Temp_Motor, Current_In, PID_Pos};
};