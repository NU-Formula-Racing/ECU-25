#pragma once

#ifdef ESP32
#include "esp_can.h"
#endif
#include "can_interface.h"
#include "virtualTimer.h"

// change specific bounds after testing with sensors in pedalbox:
enum class Bounds {
    APPS1_RAW_MIN = 1456,
    APPS1_RAW_MAX = 4095,
    APPS1_RAW_SPAN = APPS1_RAW_MAX - APPS1_RAW_MIN,
    APPS2_RAW_MIN = 1456,
    APPS2_RAW_MAX = 4095,
    APPS2_RAW_SPAN = APPS2_RAW_MAX - APPS2_RAW_MIN,
    FRONT_BRAKE_RAW_MIN = 1456,
    FRONT_BRAKE_RAW_MAX = 4095,
    FRONT_BRAKE_RAW_SPAN = FRONT_BRAKE_RAW_MAX - FRONT_BRAKE_RAW_MIN,
    REAR_BRAKE_RAW_MIN = 1456,
    REAR_BRAKE_RAW_MAX = 4095,
    REAR_BRAKE_RAW_SPAN = REAR_BRAKE_RAW_MAX - REAR_BRAKE_RAW_MIN,
    FRONT_BRAKE_RAW_PRESSED_THRESHOLD = 1000
    
};

enum class BrakeStatus {
    VALID = 1,
    INVALID = 0
};

class ThrottleBrake {
    // we should have a larger conversation about scope/access control (what functions make sense as public vs private)
    // functions we want accessible outside of this class (ie. called in main): 
    // initialize(), get_APPS1(), is_brake_pressed(), is_implausibility_present(), send_throttle_brake_CAN()
    // functions we should make private (ie. not called anywhere but in the throttle/brake class):
    // read_ADCs(), get_APPS2(), get_front_brake(), get_rear_brake(), is_brake_implausible(), is_10_percent_rule_implausible(), is_BPPC_implausible() 
    public:
        ThrottleBrake(ICAN &can_interface_, VirtualTimer &APPSs_differ_by_10_percent_timer_, VirtualTimer &brake_opened_or_shorted_timer_) : CAN_interface(can_interface_), APPSs_differ_by_10_percent_timer(APPSs_differ_by_10_percent_timer_), brake_opened_or_shorted_timer(brake_opened_or_shorted_timer_) {};
        // init timers in initialize()
        // we want 1 single-use timer per implausibility check
        // we'll have a callback function fires when the timer reaches its limit (ie. 100ms)
        // this callback will set a corresponding implausibility flag in a private struct containting all the implausibilities
        void initialize();
        int32_t get_APPS1();

        void check_APPSs_differing_by_10_percent_implausibility();
        void check_brake_opened_or_shorted_implausibility();

        bool is_brake_pressed();
        bool is_implausibility_present();
        void update_throttle_brake_CAN_signals();

        void set_brake_opened_or_shorted_implausibility();
        void set_APPSs_differ_by_10_percent_implausibility();

    private:
        void initialize_CS_pin(uint8_t CS_pin);
        void initialize_CS_pins(std::vector<uint8_t> CS_pins);
        void read_from_SPI_ADC(uint8_t CS_pin, int32_t* raw_sensor_value_class_variable);
        void read_from_SPI_ADCs(std::vector<std::pair<uint8_t, int32_t*>> CS_pins_and_raw_sensor_value_class_variables);
        void read_ADCs();
        
        ICAN &CAN_interface;
        VirtualTimer &APPSs_differ_by_10_percent_timer;
        VirtualTimer &brake_opened_or_shorted_timer;

        int32_t APPS1_raw; // raw value from APPS1 sensor in ADC counts
        int32_t APPS1; // APPS1 value SCALED from 0-32767
        int32_t APPS2_raw; // raw value from APPS2 sensor in ADC counts
        int32_t APPS2; // APPS2 value SCALED from 0-32767
        int32_t front_brake_raw;
        int32_t front_brake;
        int32_t rear_brake_raw;
        int32_t rear_brake;
        bool brake_pressed;

        bool BPPC_implausibility_present;
        bool brake_opened_or_shorted_implausibility_present;
        bool APPSs_differ_by_10_percent_implausibility_present;

        void ADC_setup(); // good that this is in private, move it out of the data members tho, into a function section. CAN should still be at the bottom of this file however
        // instead of using these (below) types of private data members + millis() to serve as timers, we should use the timer library (more comments about this above)
        bool is_brake_implausible();
        bool is_10_percent_rule_implausible();
        void check_BPPC_implausibility();
        int32_t get_front_brake();
        int32_t get_rear_brake();
        const uint32_t kTransmissionID = 0x111; // CAN msg address, get this from DBC
        // CAN signals & msgs 
        // tx: throttle percent, front brake, rear brake, brake pressed, implausibility present
        CANSignal<int32_t, 0, 32, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> throttle_percent{};
        CANSignal<int32_t, 32, 32, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> front_brake_pressure{};
        CANSignal<int32_t, 32, 32, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> rear_brake_pressure{};
        CANSignal<bool, 48, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> brake_pressed_signal{};
        CANSignal<bool, 56, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> implausibility_present_signal{};
        CANTXMessage<5> throttle_brake_data{
            CAN_interface, kTransmissionID, 8, 100,
            throttle_percent, front_brake_pressure, rear_brake_pressure, brake_pressed_signal, implausibility_present_signal
        };
};
