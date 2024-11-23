#pragma once

#ifdef ESP32
#include "esp_can.h"
#endif
#include "can_interface.h"
#include "virtualTimer.h"

class ThrottleBrake {
    // we should have a larger conversation about scope/access control (what functions make sense as public vs private)
    // functions we want accessible outside of this class (ie. called in main): 
    // initialize(), get_APPS1(), is_brake_pressed(), is_implausibility_present(), send_throttle_brake_CAN()
    // functions we should make private (ie. not called anywhere but in the throttle/brake class):
    // read_ADCs(), get_APPS2(), get_front_brake(), get_rear_brake(), is_brake_implausible(), is_10_percent_rule_implausible(), is_BPPC_implausible() 
    public:
        // add a pass by reference input to the constructor for a throttle/brake timer group
        ThrottleBrake(ICAN &can_interface_, VirtualTimerGroup &timers);
        // init timers in initialize()
        // we want 1 single-use timer per implausibility check
        // we'll have a callback function fires when the timer reaches its limit (ie. 100ms)
        // this callback will set a corresponding implausibility flag in a private struct containting all the implausibilities
        void initialize();
        int32_t get_APPS1();
        bool is_brake_pressed();
        bool is_implausibility_present();
        void send_throttle_brake_CAN();

    private:
        void initialize_CS_pin(uint8_t CS_pin);
        void initialize_CS_pins(std::vector<uint8_t> CS_pins);
        
        ICAN &can_interface;
        int32_t APPS1_raw; // raw value from APPS1 sensor in ADC counts
        int32_t APPS1; // APPS1 value SCALED from 0-32767
        int32_t APPS2_raw; // raw value from APPS2 sensor in ADC counts
        int32_t APPS2; // APPS2 value SCALED from 0-32767
        int32_t front_brake_raw;
        int32_t front_brake;
        int32_t rear_brake_raw;
        int32_t rear_brake;
        bool brake_pressed;
        bool implausibility_present;
        bool brake_valid;
        bool throttle_dropping_to_5_percent_after_brake_implausibility = false;
        void ADC_setup(); // good that this is in private, move it out of the data members tho, into a function section. CAN should still be at the bottom of this file however
        // instead of using these (below) types of private data members + millis() to serve as timers, we should use the timer library (more comments about this above)
        void readADCs();
        bool is_brake_implausible();
        bool is_10_percent_rule_implausible();
        bool is_BPPC_implausible();
        int32_t get_front_brake();
        int32_t get_rear_brake();
        long time_of_start_of_ten_percent_implasibility = 0;
        long time_of_start_of_brake_implausibility = 0;
        const uint32_t kTransmissionID = 0x111; // CAN msg address, get this from DBC
        // CAN signals & msgs 
        // tx: throttle percent, front brake, rear brake, brake pressed, implausibility present
        CANSignal<int32_t, 0, 32, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> throttle_percent{};
        CANSignal<int32_t, 32, 32, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> front_brake_pressure{};
        CANSignal<int32_t, 32, 32, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> rear_brake_pressure{};
        CANSignal<bool, 48, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> brake_pressed_signal{};
        CANSignal<bool, 56, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> implausibility_present_signal{};
        CANTXMessage<5> throttle_brake_data{
            can_interface, kTransmissionID, 8, 100,
            throttle_percent, front_brake_pressure, rear_brake_pressure, brake_pressed_signal, implausibility_present_signal
        };
};
