#pragma once

#ifdef ESP32
#include "esp_can.h"
#endif

#include "can_interface.h"
#include "virtualTimer.h"

// change specific bounds after testing with sensors in pedalbox:
enum class Bounds {

    APPS1_DIGITAL_SIGNAL_MIN = 1456,
    APPS1_DIGITAL_SIGNAL_MAX = 4095,
    APPS1_DIGITAL_SIGNAL_SPAN = APPS1_DIGITAL_SIGNAL_MAX - APPS1_DIGITAL_SIGNAL_MIN,

    APPS2_DIGITAL_SIGNAL_MIN = 1456,
    APPS2_DIGITAL_SIGNAL_MAX = 4095,
    APPS2_DIGITAL_SIGNAL_SPAN = APPS2_DIGITAL_SIGNAL_MAX - APPS2_DIGITAL_SIGNAL_MIN,

    FRONT_BRAKE_DIGITAL_SIGNAL_MIN = 1456,
    FRONT_BRAKE_DIGITAL_SIGNAL_MAX = 4095,
    FRONT_BRAKE_DIGITAL_SIGNAL_SPAN = FRONT_BRAKE_DIGITAL_SIGNAL_MAX - FRONT_BRAKE_DIGITAL_SIGNAL_MIN,

    REAR_BRAKE_DIGITAL_SIGNAL_MIN = 1456,
    REAR_BRAKE_DIGITAL_SIGNAL_MAX = 4095,
    REAR_BRAKE_DIGITAL_SIGNAL_SPAN = REAR_BRAKE_DIGITAL_SIGNAL_MAX - REAR_BRAKE_DIGITAL_SIGNAL_MIN,

    FRONT_BRAKE_DIGITAL_SIGNAL_PRESSED_THRESHOLD = 1000,

    FIVE_PERCENT_THROTTLE = 1639,
    TEN_PERCENT_THROTTLE = 3276,
    TWENTY_FIVE_PERCENT_THROTTLE = 8191

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
        ThrottleBrake(ICAN &can_interface_, VirtualTimer &APPSs_disagreement_implausibility_timer_, VirtualTimer &brake_shorted_or_opened_implausibility_timer_) : CAN_interface(can_interface_), APPSs_disagreement_implausibility_timer(APPSs_disagreement_implausibility_timer_), brake_shorted_or_opened_implausibility_timer(brake_shorted_or_opened_implausibility_timer_) {};
        // init timers in initialize()
        // we want 1 single-use timer per implausibility check
        // we'll have a callback function fires when the timer reaches its limit (ie. 100ms)
        // this callback will set a corresponding implausibility flag in a private struct containting all the implausibilities
        void initialize();
        void set_APPSs_disagreement_implausibility_present_to_true();
        void set_brake_shorted_or_opened_implausibility_present_to_true();
        void update_sensor_values();
        int16_t get_APPS1_throttle();
        bool is_implausibility_present();
        void check_brake_shorted_or_opened_implausibility();
        void check_APPSs_disagreement_implausibility();
        bool is_brake_pressed();
        void update_throttle_brake_CAN_signals();

    private:
        int16_t APPS1_digital_signal; // 12-bit ADC: 0-4095
        int16_t APPS2_digital_signal; // 12-bit ADC: 0-4095
        int16_t front_break_digital_signal; // 12-bit ADC: 0-4095
        int16_t rear_break_digital_signal; // 12-bit ADC: 0-4095

        int16_t APPS1_throttle; // throttle calculated from APPS1 and scaled 0-32767
        int16_t APPS2_throttle; // throttle calculated from APPS2 and scaled 0-32767
        int16_t front_brake; // front brake scaled 0-32767
        int16_t rear_brake; // rear brake scaled 0-32767

        bool implausibility_present;
        bool APPSs_disagreement_implausibility_present;
        bool pedal_misapplication_implausibility_present;
        bool brake_shorted_or_opened_implausibility_present;

        void set_implausibilities_to_false();
        void read_from_SPI_ADC(int8_t CS_pin, int16_t ThrottleBrake::*digital_signal);
        void read_from_SPI_ADCs();

        VirtualTimer &APPSs_disagreement_implausibility_timer;
        VirtualTimer &brake_shorted_or_opened_implausibility_timer;

        bool brake_pressed;

        void check_pedal_misapplication_implausibility();

        ICAN &CAN_interface;
        const uint32_t kTransmissionID = 0x111; // CAN msg address, get this from DBC
        // CAN signals & msgs 
        // tx: throttle percent, front brake, rear brake, brake pressed, implausibility present
        CANSignal<int16_t, 0, 32, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> throttle_signal{};
        CANSignal<int16_t, 32, 32, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> front_brake_signal{};
        CANSignal<int16_t, 32, 32, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> rear_brake_signal{};
        CANSignal<bool, 48, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> brake_pressed_signal{};
        CANSignal<bool, 56, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> implausibility_present_signal{};
        CANTXMessage<5> throttle_brake_data{
            CAN_interface, kTransmissionID, 8, 100,
            throttle_signal, front_brake_signal, rear_brake_signal, brake_pressed_signal, implausibility_present_signal
        };
};
