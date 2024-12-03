#pragma once

#include "esp_can.h"

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

    // FIVE_PERCENT_THROTTLE = 1639,
    // TEN_PERCENT_THROTTLE = 3276,
    // TWENTY_FIVE_PERCENT_THROTTLE = 8191

};

enum class BrakeStatus {
    VALID = 1,
    INVALID = 0
};

class ThrottleBrake {
    // functions we want accessible outside of this class (ie. called in main): 
    // initialize(), get_APPS1(), is_brake_pressed(), is_implausibility_present(), send_throttle_brake_CAN()
    // functions we should make private (ie. not called anywhere but in the throttle/brake class):
    // read_ADCs(), get_APPS2(), get_front_brake(), get_rear_brake(), is_brake_implausible(), is_10_percent_rule_implausible(), is_BPPC_implausible() 
    public:
        ThrottleBrake(ICAN &can_interface_, VirtualTimer &APPSs_disagreement_implausibility_timer_, VirtualTimer &brake_shorted_or_opened_implausibility_timer_); 
    
        // init timers in initialize()
        // we want 1 single-use timer per implausibility check
        // we'll have a callback function fires when the timer reaches its limit (ie. 100ms)
        // this callback will set a corresponding implausibility flag in a private struct containting all the implausibilities
        void initialize();
        void update_sensor_values();
        int16_t get_throttle();
        void set_is_APPSs_disagreement_implausibility_present_to_true(); // callback
        void set_is_brake_shorted_or_opened_implausibility_present_to_true(); // callback
        void check_for_implausibilities();
        bool is_implausibility_present();
        bool is_brake_pressed();
        void update_throttle_brake_CAN_signals();

    private:
        int16_t APPS1_RAW; // 12-bit ADC: 0-4095
        int16_t APPS2_RAW; // 12-bit ADC: 0-4095
        int16_t front_break_RAW; // 12-bit ADC: 0-4095
        int16_t rear_break_RAW; // 12-bit ADC: 0-4095

        int16_t APPS1_throttle; // throttle calculated from APPS1 and scaled 0-32767
        int16_t APPS2_throttle; // throttle calculated from APPS2 and scaled 0-32767
        int16_t front_brake; // front brake scaled 0-32767
        int16_t rear_brake; // rear brake scaled 0-32767

        bool APPSs_disagreement_implausibility_present;
        bool BPPC_implausibility_present;
        bool brake_shorted_or_opened_implausibility_present; 

        void read_from_SPI_ADCs();

        VirtualTimer &APPSs_disagreement_implausibility_timer;
        VirtualTimer &brake_shorted_or_opened_implausibility_timer;

        bool brake_pressed;

        void initialize_CS_pin(uint8_t CS_pin);
        void initialize_CS_pins();

        int16_t get_safe_RAW(int16_t RAW, int16_t projected_min, int16_t projected_max);

        void check_BPPC_implausibility();
        void check_brake_shorted_or_opened_implausibility();
        void check_APPSs_disagreement_implausibility();

        ICAN &CAN_interface;
        const uint32_t kTransmissionID = 0x111; // CAN msg address, get this from DBC
        // CAN signals & msgs 
        // tx: throttle percent, front brake, rear brake, brake pressed, implausibility present
        CANSignal<int32_t, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> APPS1_throttle_signal{};
        CANSignal<int32_t, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> APPS2_throttle_signal{};
        CANSignal<int32_t, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> front_brake_signal{};
        CANSignal<int32_t, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> rear_brake_signal{};
        CANSignal<bool, 32, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> brake_pressed_signal{};
        CANSignal<bool, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> implausibility_present_signal{};
        // CANTXMessage<6> throttle_brake_data{
        //     CAN_interface, kTransmissionID, 10, 100, 
        //     APPS1_throttle_signal, APPS2_throttle_signal, front_brake_signal, rear_brake_signal, brake_pressed_signal, implausibility_present_signal
        // };
        CANTXMessage<2> throttle_data{
            CAN_interface, kTransmissionID, 10, 100,
            APPS1_throttle_signal, APPS2_throttle_signal
        };
        CANTXMessage<3> brake_data{
            CAN_interface, kTransmissionID, 10, 100,
            front_brake_signal, rear_brake_signal, brake_pressed_signal
        };
        CANTXMessage<1> implausibility_data{
            CAN_interface, kTransmissionID, 10, 100,
            implausibility_present_signal
        };
};
