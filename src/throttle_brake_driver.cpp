#include <Arduino.h>
#include <SPI.h>

#include "throttle_brake_driver.hpp"
#include "pins.hpp"


void initialize_CS_pin(int8_t CS_pin) {
    pinMode(CS_pin, OUTPUT);
    digitalWrite(CS_pin, HIGH);
}

void initialize_CS_pins() {
    initialize_CS_pin(static_cast<int8_t>(Pins::APPS1_CS_PIN));
    initialize_CS_pin(static_cast<int8_t>(Pins::APPS2_CS_PIN));
    initialize_CS_pin(static_cast<int8_t>(Pins::FRONT_BRAKE_CS_PIN));
    initialize_CS_pin(static_cast<int8_t>(Pins::REAR_BRAKE_CS_PIN));
}

void ThrottleBrake::set_implausibilities_to_false() {

    ThrottleBrake::implausibility_present = false;

    ThrottleBrake::APPSs_disagreement_implausibility_present = false;
    ThrottleBrake::pedal_misapplication_implausibility_present = false;
    ThrottleBrake::brake_shorted_or_opened_implausibility_present = false;

}

/**
 * @brief Set implausibilities false, start SPI bus, set pin modes, write default HIGH to CS pins
 */
void ThrottleBrake::initialize() {
    ThrottleBrake::set_implausibilities_to_false();
    SPI.begin();
    initialize_CS_pins();
    pinMode(static_cast<int8_t>(Pins::BRAKE_VALID_PIN), INPUT);
};

void ThrottleBrake::set_APPSs_disagreement_implausibility_present_to_true() { 
    ThrottleBrake::APPSs_disagreement_implausibility_present = true;
    ThrottleBrake::implausibility_present = true;
}

void ThrottleBrake::set_brake_shorted_or_opened_implausibility_present_to_true() {
    ThrottleBrake::brake_shorted_or_opened_implausibility_present = true;
    ThrottleBrake::implausibility_present = true;
}

void ThrottleBrake::read_from_SPI_ADC(int8_t CS_pin, int16_t ThrottleBrake::*digital_signal) {
    digitalWrite(CS_pin, LOW);
    this->*digital_signal = (SPI.transfer32(0x0000) << 2) >> 4;
    digitalWrite(CS_pin, HIGH);
}

/**
 * @brief Reads data from ADCs and stores RAW sensor data (in ADC counts) in class variables  
 */
void ThrottleBrake::read_from_SPI_ADCs() {
    
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
    // 1000000: clock rate
    // MSBFIRST: Most significant bit first

    read_from_SPI_ADC(static_cast<int8_t>(Pins::APPS1_CS_PIN), &ThrottleBrake::APPS1_digital_signal);
    read_from_SPI_ADC(static_cast<int8_t>(Pins::APPS2_CS_PIN), &ThrottleBrake::APPS2_digital_signal);
    read_from_SPI_ADC(static_cast<int8_t>(Pins::FRONT_BRAKE_CS_PIN), &ThrottleBrake::front_break_digital_signal);
    read_from_SPI_ADC(static_cast<int8_t>(Pins::REAR_BRAKE_CS_PIN), &ThrottleBrake::rear_break_digital_signal);

    SPI.endTransaction();
}

int16_t get_safe_digital_signal(int16_t digital_signal, int16_t projected_min, int16_t projected_max) {
    if (digital_signal < projected_min) {
        return projected_min;
    } else if (digital_signal > projected_max) {
        return projected_max;
    } else {
        return digital_signal;
    }
}

void ThrottleBrake::update_sensor_values() {

    ThrottleBrake::read_from_SPI_ADCs();

    int16_t safe_APPS1_digital_signal = get_safe_digital_signal(ThrottleBrake::APPS1_digital_signal, static_cast<int16_t>(Bounds::APPS1_DIGITAL_SIGNAL_MIN), static_cast<int16_t>(Bounds::APPS1_DIGITAL_SIGNAL_MAX));
    ThrottleBrake::APPS1_throttle = (safe_APPS1_digital_signal - static_cast<int16_t>(Bounds::APPS1_DIGITAL_SIGNAL_MIN)) / static_cast<int16_t>(Bounds::APPS1_DIGITAL_SIGNAL_SPAN) * 32767;

    int16_t safe_APPS2_digital_signal = get_safe_digital_signal(ThrottleBrake::APPS2_digital_signal, static_cast<int16_t>(Bounds::APPS2_DIGITAL_SIGNAL_MIN), static_cast<int16_t>(Bounds::APPS2_DIGITAL_SIGNAL_MAX));
    ThrottleBrake::APPS2_throttle = (static_cast<int16_t>(Bounds::APPS2_DIGITAL_SIGNAL_MAX) - safe_APPS2_digital_signal) / static_cast<int16_t>(Bounds::APPS2_DIGITAL_SIGNAL_SPAN) * 32767;

    int16_t safe_front_break_digital_signal = get_safe_digital_signal(ThrottleBrake::front_break_digital_signal, static_cast<int16_t>(Bounds::FRONT_BRAKE_DIGITAL_SIGNAL_MIN), static_cast<int16_t>(Bounds::FRONT_BRAKE_DIGITAL_SIGNAL_MAX));
    ThrottleBrake::front_brake = (safe_front_break_digital_signal - static_cast<int16_t>(Bounds::FRONT_BRAKE_DIGITAL_SIGNAL_MIN)) / static_cast<int16_t>(Bounds::FRONT_BRAKE_DIGITAL_SIGNAL_SPAN) * 32767;

    int16_t safe_rear_break_digital_signal = get_safe_digital_signal(ThrottleBrake::rear_break_digital_signal, static_cast<int16_t>(Bounds::REAR_BRAKE_DIGITAL_SIGNAL_MIN), static_cast<int16_t>(Bounds::REAR_BRAKE_DIGITAL_SIGNAL_MAX));
    ThrottleBrake::rear_brake = (safe_rear_break_digital_signal - static_cast<int16_t>(Bounds::REAR_BRAKE_DIGITAL_SIGNAL_MIN)) / static_cast<int16_t>(Bounds::REAR_BRAKE_DIGITAL_SIGNAL_SPAN) * 32767;

}

/**
 * @brief Returns APPS1 throttle value, scaled 0-32767
 *
 * @return int16_t
 */   
int16_t ThrottleBrake::get_APPS1_throttle() {
    return ThrottleBrake::APPS1_throttle;
};

/**
 * @brief Returns true if any implausibility is present, false otherwise
 *
 * @return bool
 */
bool ThrottleBrake::is_implausibility_present() {
    return ThrottleBrake::implausibility_present;
};
        
/**
 * @brief Checks if brake is implausible (open or shorted for >100ms). (T.4.3.3)
 *
 * @return void
 */     
void ThrottleBrake::check_brake_shorted_or_opened_implausibility() {
    if (digitalRead(static_cast<int8_t>(Pins::BRAKE_VALID_PIN)) == static_cast<bool>(BrakeStatus::INVALID)) {
        if (ThrottleBrake::brake_shorted_or_opened_implausibility_timer.GetTimerState() == VirtualTimer::State::kNotStarted) {
            ThrottleBrake::brake_shorted_or_opened_implausibility_present = true;
            ThrottleBrake::implausibility_present = true;
            ThrottleBrake::brake_shorted_or_opened_implausibility_timer.Start(millis());
        }
        // if implausibility detected and timer already running, then do nothing (timer ticks in main)
    } else if (ThrottleBrake::brake_shorted_or_opened_implausibility_timer.GetTimerState() == VirtualTimer::State::kRunning) {
        ThrottleBrake::brake_shorted_or_opened_implausibility_present = false;
        ThrottleBrake::implausibility_present = false;
        ThrottleBrake::brake_shorted_or_opened_implausibility_timer.Disable();
        ThrottleBrake::brake_shorted_or_opened_implausibility_timer.Enable();
    } // if there is no implausibility detected and no current implausibility timer running, then do nothing
};

int16_t get_APPSs_throttle_difference(int16_t APPS1_throttle, int16_t APPS2_throttle) {
    int16_t APPS1_throttle_minus_APPS2_throttle = APPS1_throttle - APPS2_throttle;
    if (APPS1_throttle_minus_APPS2_throttle < 0) {
        return APPS1_throttle_minus_APPS2_throttle * -1;
    } else {
        return APPS1_throttle_minus_APPS2_throttle;
    }
}

/**
 * @brief Returns true if APPS1 and APPS2 disagree by >10% for >100ms, 
 *        Returns False otherwise
 *        (T.4.2.4 - T.4.2.5)
 *
 * @return void
 */
void ThrottleBrake::check_APPSs_disagreement_implausibility() {

    int16_t APPSs_throttle_difference = get_APPSs_throttle_difference(ThrottleBrake::APPS1_throttle, ThrottleBrake::APPS2_throttle);
    if (APPSs_throttle_difference > static_cast<int16_t>(Bounds::TEN_PERCENT_THROTTLE)) {
        if (ThrottleBrake::APPSs_disagreement_implausibility_timer.GetTimerState() == VirtualTimer::State::kNotStarted) {
            ThrottleBrake::APPSs_disagreement_implausibility_present = true;
            ThrottleBrake::implausibility_present = true;
            ThrottleBrake::APPSs_disagreement_implausibility_timer.Start(millis());
        }
        // if implausibility detected and timer already running, then do nothing (timer ticks in main)
    } else if (ThrottleBrake::APPSs_disagreement_implausibility_timer.GetTimerState() == VirtualTimer::State::kRunning) {
        ThrottleBrake::APPSs_disagreement_implausibility_present = false;
        ThrottleBrake::implausibility_present = false;
        ThrottleBrake::APPSs_disagreement_implausibility_timer.Disable();
        ThrottleBrake::APPSs_disagreement_implausibility_timer.Enable();
    }
    // if there is no implausibility detected and no current implausibility timer running, then do nothing

    // we need to worry about what happens if the timer actually expires?


    // // Convert APPS raw values to throttle percentages (0-100)
    // float APPS1_percentage = ((ThrottleBrake::APPS1_raw - static_cast<int32_t>(Bounds::APPS1_RAW_MIN)) * 100.0) / static_cast<int32_t>(Bounds::APPS1_RAW_SPAN);
    // float APPS2_percentage = ((static_cast<int32_t>(Bounds::APPS2_RAW_MAX) - ThrottleBrake::APPS2_raw) * 100.0) / static_cast<int32_t>(Bounds::APPS2_RAW_SPAN);
    // int32_t APPS_diff = APPS1_percentage - APPS2_percentage; // Get percentage point difference between APPS values
    // if ((APPS_diff > 10.0 || APPS_diff < -10.0) && ThrottleBrake::APPSs_differ_by_10_percent_timer.GetTimerState() == VirtualTimer::State::kNotStarted) {
    //     ThrottleBrake::APPSs_differ_by_10_percent_timer.Start(millis());
        
    // } else if (!(APPS_diff > 10.0 || APPS_diff < -10.0) && ThrottleBrake::APPSs_differ_by_10_percent_timer.GetTimerState() == VirtualTimer::State::kRunning) {
    //     ThrottleBrake::APPSs_differ_by_10_percent_timer.Disable();
    //     ThrottleBrake::APPSs_differ_by_10_percent_timer.Enable();
    // }

};

/**
 * @brief Returns true if brake is pressed, false otherwise
 *
 * @return bool
 */
bool ThrottleBrake::is_brake_pressed() {
    // check if the front brake value is over a certain threshold (actual threshold is TBD, need to test with brake sensors)
    // if yes: return true
    // if no: return false
    if (ThrottleBrake::front_break_digital_signal >= static_cast<int32_t>(Bounds::FRONT_BRAKE_DIGITAL_SIGNAL_PRESSED_THRESHOLD)) {
        ThrottleBrake::brake_pressed = true;
        return true;
    }
    ThrottleBrake::brake_pressed = false;
    return false;
};

/**
 * @brief Sets true if both brake is pressed and throttle is >25%, 
 *        Sets false after throttle returns to <=5% (regardless of brake) or otherwise 
 *        (EV.4.7)
 *
 * @return 
 */
void ThrottleBrake::check_pedal_misapplication_implausibility() {

    if (ThrottleBrake::brake_pressed && ThrottleBrake::APPS1_throttle > static_cast<int16_t>(Bounds::TWENTY_FIVE_PERCENT_THROTTLE)) {
        ThrottleBrake::pedal_misapplication_implausibility_present = true;
        ThrottleBrake::implausibility_present = true;
    } else if (ThrottleBrake::APPS1_throttle < static_cast<int16_t>(Bounds::FIVE_PERCENT_THROTTLE)) {
        ThrottleBrake::pedal_misapplication_implausibility_present = false;
        ThrottleBrake::implausibility_present = false;
    }


    // float APPS1_percentage = ((ThrottleBrake::APPS1_raw - static_cast<int32_t>(Bounds::APPS1_RAW_MIN)) * 100.0) / static_cast<int32_t>(Bounds::APPS1_RAW_SPAN);
    // if (ThrottleBrake::is_brake_pressed() && (APPS1_percentage > 25.0)) {
    //     ThrottleBrake::pedal_misapplication_implausibility_present = true;
    // }
    // if (APPS1_percentage < 5.0) {
    //     ThrottleBrake::pedal_misapplication_implausibility_present = false;
    // }

};

/**
 * @brief Check for implausibility and set throttle/brake CAN signals accordingly
 *
 * @return void
 */
void ThrottleBrake::update_throttle_brake_CAN_signals() {
    ThrottleBrake::throttle_signal = ThrottleBrake::APPS1_throttle;
    ThrottleBrake::front_brake_signal = ThrottleBrake::front_brake;
    ThrottleBrake::rear_brake_signal = ThrottleBrake::rear_brake;
    ThrottleBrake::brake_pressed_signal = ThrottleBrake::brake_pressed;
    ThrottleBrake::implausibility_present_signal = ThrottleBrake::implausibility_present;
};
