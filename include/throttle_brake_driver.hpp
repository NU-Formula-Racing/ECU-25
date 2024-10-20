#pragma once

#include "esp_can.h"
#include "can_interface.h"
#include "virtualTimer.h"

// #define MAX_THROTTLE 32767

class ThrottleBrake {
    public:
        ThrottleBrake(ICAN &can_interface_) : can_interface(can_interface_){};
        void initialize();
        void read_ADCs();
        uint16_t get_APPS1();
        uint16_t get_APPS2();
        int16_t get_front_brake();
        int16_t get_rear_brake();
        bool is_brake_pressed();
        bool is_implausibility_present();
        bool is_brake_implausible();
        bool is_10_percent_rule_implausible();
        bool is_BPPC_implausible();
        void send_throttle_brake_CAN();
    private:
        ICAN &can_interface;
        uint16_t APPS1_raw; // raw value from APPS1 sensor in ADC counts
        uint16_t APPS2_raw; 
        uint16_t front_brake_raw;
        uint16_t rear_brake_raw;
        uint16_t APPS1; // APPS1 value SCALED from 0-32767
        uint16_t APPS2; // APPS2 value SCALED from 0-32767
        uint16_t front_brake;
        uint16_t rear_brake;
        bool brake_pressed;
        bool implausibility_present;
        bool brake_valid;
        bool throttle_dropping_to_5_percent_after_brake_implausibility = false;
        void ADC_setup();
        long time_of_start_of_ten_percent_implasibility = 0;
        long time_of_start_of_brake_implausibility = 0;
        const uint16_t kTransmissionID = 0x111; // CAN msg address, get this from DBC
        // CAN signals & msgs 
        // tx: throttle percent, front brake, rear brake, brake pressed, implausibility present
        CANSignal<int16_t, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> throttle_percent{};
        CANSignal<int16_t, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> front_brake_pressure{};
        CANSignal<int16_t, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> rear_brake_pressure{};
        CANSignal<bool, 48, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> brake_pressed_signal{};
        CANSignal<bool, 56, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> implausibility_present_signal{};
        CANTXMessage<5> throttle_brake_data{
            can_interface, kTransmissionID, 8, 100,
            throttle_percent, front_brake_pressure, rear_brake_pressure, brake_pressed_signal, implausibility_present_signal
        };
};
