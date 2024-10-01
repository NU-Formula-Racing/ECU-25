#pragma once

#include <Arduino.h>
#include "esp_can.h"
#include "can_interface.h"

// #define MAX_THROTTLE 32767

// change specific bounds after testing with sensors in pedalbox
enum class Bounds {
    APPS1_MIN = 0,
    APPS1_MAX = 32767,
    APPS2_MIN = 0,
    APPS2_MAX = 32767,
    FRONT_BRAKE_MIN = 0,
    FRONT_BRAKE_MAX = 32767,
    REAR_BRAKE_MIN = 0,
    REAR_BRAKE_MAX = 32767
};

// use GPIO names
enum class Pins {
    APPS1_CS_PIN = 0,
    APPS2_CS_PIN = 0,
    FRONT_BRAKE_CS_PIN = 0,
    REAR_BRAKE_CS_PIN = 0,
    BRAKE_VALID_PIN = 0,
    DRIVE_LEVER_PIN = 0
};

class ThrottleBrake {
    public:
        ThrottleBrake(ICAN &can_interface_) : can_interface(can_interface_){};
        void initialize();
        void read_ADCs();
        int16_t get_APPS1(); // APPS1 value SCALED from 0-32767
        int16_t get_APPS2();
        int16_t get_front_brake();
        int16_t get_rear_brake();
        bool is_brake_pressed();
        bool is_implausibility_present();
        bool is_brake_valid();
        void check_ten_percent_rule();
        void check_BPPC_rule();
    private:
        ICAN &can_interface;
        int16_t APPS1_value; // raw value from APPS1 sensor in ADC counts
        int16_t APPS2_value; 
        int16_t front_brake_value;
        int16_t rear_brake_value;
        bool brake_pressed;
        bool implausibility_present;
        bool brake_valid;
        const uint16_t kTransmissionID = 0x111; // CAN msg address, get this from DBC
        // CAN signals & msgs 
        // tx: throttle percent, front brake, rear brake, brake pressed, implausibility present
        CANSignal<int16_t, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> throttle_percent{};
        CANSignal<int16_t, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> front_brake_pressure{};
        CANSignal<int16_t, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> rear_brake_pressure{};
        CANSignal<bool, 48, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> brake_pressed{};
        CANSignal<bool, 56, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> implausibility_present{};
        CANTXMessage<5> throttle_brake_data{
            drive_bus, kTransmissionID, 8, 100, timers, 
            throttle_percent, front_brake_pressure, rear_brake_pressure, brake_pressed, implausibility_present};
};