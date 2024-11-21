#pragma once

#ifdef ESP32
#include "esp_can.h"
#endif
#include "can_interface.h"
#include "virtualTimer.h"

// #define MAX_THROTTLE 32767 // delete this now we dont need it 

class ThrottleBrake {
    // we should have a larger conversation about scope/access control (what functions make sense as public vs private)
    // functions we want accessible outside of this class (ie. called in main): 
    // initialize(), get_APPS1(), is_brake_pressed(), is_implausibility_present(), send_throttle_brake_CAN()
    // functions we should make private (ie. not called anywhere but in the throttle/brake class):
    // read_ADCs(), get_APPS2(), get_front_brake(), get_rear_brake(), is_brake_implausible(), is_10_percent_rule_implausible(), is_BPPC_implausible() 
    public:
        // get rid of the test input here lol
        // add a pass by reference input to the constructor for a throttle/brake timer group 
        ThrottleBrake(ICAN &can_interface_, uint16_t test_front_brake);
        // init timers in initialize()
        // we want 1 single-use timer per implausibility check
        // we'll have a callback function fires when the timer reaches its limit (ie. 100ms)
        // this callback will set a corresponding implausibility flag in a private struct containting all the implausibilities
        void initialize();
        uint16_t get_APPS1();
        uint16_t get_APPS2();
        uint16_t get_front_brake();
        uint16_t get_rear_brake();
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
        void ADC_setup(); // good that this is in private, move it out of the data members tho, into a function section. CAN should still be at the bottom of this file however
        // instead of using these (below) types of private data members + millis() to serve as timers, we should use the timer library (more comments about this above)
        void readADCs();
        bool is_brake_pressed();
        bool is_implausibility_present();
        bool is_brake_implausible();
        bool is_10_percent_rule_implausible();
        bool is_BPPC_implausible();
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
