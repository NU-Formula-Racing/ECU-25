#include <Arduino.h>
#include <SPI.h>

#include "throttle_brake_driver.hpp"
#include "pins.hpp"



void ThrottleBrake::initialize_CS_pin(uint8_t CS_pin) {
    pinMode(CS_pin, OUTPUT);
    digitalWrite(CS_pin, HIGH);
}

void ThrottleBrake::initialize_CS_pins(std::vector<uint8_t> CS_pins) {
    for (uint8_t CS_pin : CS_pins) {
        ThrottleBrake::initialize_CS_pin(CS_pin);
    }
}

/**
 * @brief Start SPI bus, set pin modes, write default HIGH to CS pins
 */
void ThrottleBrake::initialize() {

    ThrottleBrake::BPPC_implausibility_present = false;

    SPI.begin();

    std::vector<uint8_t> CS_pins = {
        static_cast<uint8_t>(Pins::APPS1_CS_PIN),
        static_cast<uint8_t>(Pins::APPS2_CS_PIN),
        static_cast<uint8_t>(Pins::FRONT_BRAKE_CS_PIN),
        static_cast<uint8_t>(Pins::REAR_BRAKE_CS_PIN)
    };
    ThrottleBrake::initialize_CS_pins(CS_pins);

    pinMode(static_cast<uint8_t>(Pins::BRAKE_VALID_PIN), INPUT);

};

void ThrottleBrake::read_from_SPI_ADC(uint8_t CS_pin, int32_t* raw_sensor_value_class_variable) {
    digitalWrite(CS_pin, LOW);
    *raw_sensor_value_class_variable = (SPI.transfer32(0x0000) << 2) >> 4;
    digitalWrite(CS_pin, HIGH);
}

void ThrottleBrake::read_from_SPI_ADCs(std::vector<std::pair<uint8_t, int32_t*>> CS_pins_and_raw_sensor_value_class_variables) {
    for (std::pair<uint8_t, int32_t*> CS_pin_and_raw_sensor_value_class_variable : CS_pins_and_raw_sensor_value_class_variables) {
        read_from_SPI_ADC(CS_pin_and_raw_sensor_value_class_variable.first, CS_pin_and_raw_sensor_value_class_variable.second);
    }
}

/**
 * @brief Reads data from ADCs and stores RAW sensor data (in ADC counts) in class variables  
 */
void ThrottleBrake::read_ADCs() {

    // 1000000: clock rate
    // MSBFIRST: Most significant bit first
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));

    std::vector<std::pair<uint8_t, int32_t&>> CS_pins_and_raw_sensor_value_class_variables = {
        {static_cast<uint8_t>(Pins::APPS1_CS_PIN), ThrottleBrake::APPS1_raw},
        {static_cast<uint8_t>(Pins::APPS2_CS_PIN), ThrottleBrake::APPS2_raw},
        {static_cast<uint8_t>(Pins::FRONT_BRAKE_CS_PIN), ThrottleBrake::front_brake_raw},
        {static_cast<uint8_t>(Pins::REAR_BRAKE_CS_PIN), ThrottleBrake::rear_brake_raw},
    };
    ThrottleBrake::read_from_SPI_ADCs(CS_pins_and_raw_sensor_value_class_variables);

    SPI.endTransaction();

};

void ThrottleBrake::set_brake_opened_or_shorted_implausibility() {
    ThrottleBrake::brake_opened_or_shorted_implausibility_present = true;
}

void ThrottleBrake::set_APPSs_differ_by_10_percent_implausibility() {
    ThrottleBrake::APPSs_differ_by_10_percent_implausibility_present = true;
}

// 'APPS_BRAKE_update' function calls 'readADCs' function and performs conversions to scaled values, storing as private data members, needs to be public

// 10% implausibility check should use 0-32767 scaled values

// calling getAPPS1 in main

/**
 * @brief Returns APPS1 value
 *
 * @return int32_t
 */   
int32_t ThrottleBrake::get_APPS1() { // use int32_t instead of int32_t in case APPS signals go negative, we don't want overflow to cause us to output massive throttle values
    return ThrottleBrake::APPS1;
};
        
/**
 * @brief Gets Front Brake value SCALED from 0-32767
 *
 * @return int32_t
 */
int32_t ThrottleBrake::get_front_brake() { // use int32_t instead of int32_t in case brake signals go negative, we don't want overflow to cause us to output massive brake values
    return ((ThrottleBrake::front_brake_raw - static_cast<int32_t>(Bounds::FRONT_BRAKE_RAW_MIN)) * 32767) / static_cast<int32_t>(Bounds::FRONT_BRAKE_RAW_SPAN);
}

// follow same strategy for all gets
        
/**
 * @brief Gets Rear Brake value SCALED from 0-32767
 *
 * @return unt32_t
 */ 
int32_t ThrottleBrake::get_rear_brake() { // use int32_t instead of int32_t in case brake signals go negative, we don't want overflow to cause us to output massive brake values
    return (ThrottleBrake::rear_brake_raw - static_cast<int32_t>(Bounds::REAR_BRAKE_RAW_MIN) * 32767 / static_cast<int32_t>(Bounds::REAR_BRAKE_RAW_SPAN; // fix casting
};
        
/**
 * @brief Returns true if any implausibility is present, false otherwise
 *
 * @return bool
 */
// implausibility check function needs to be called periodically (ie. every 10ms)
// add a timer in init() for this purpose
// implausibility check function should be the callback
bool ThrottleBrake::is_implausibility_present() {
    // if front brake or APPS fail any implausibility checks, return true
    return (ThrottleBrake::BPPC_implausibility_present || ThrottleBrake::brake_opened_or_shorted_implausibility_present || ThrottleBrake::APPSs_differ_by_10_percent_implausibility_present);
};
        
/**
 * @brief Returns true if brake is implausible (open or shorted for >100ms), 
 *        Returns false otherwise
 *        (T.4.3.3)
 *
 * @return bool
 */     
void ThrottleBrake::check_brake_opened_or_shorted_implausibility() {

    if (digitalRead(static_cast<uint8_t>(Pins::BRAKE_VALID_PIN)) == static_cast<bool>(BrakeStatus::INVALID) && ThrottleBrake::brake_opened_or_shorted_timer.GetTimerState() == VirtualTimer::State::kNotStarted) {
        ThrottleBrake::brake_opened_or_shorted_timer.Start(millis());

    } else if (digitalRead(static_cast<uint8_t>(Pins::BRAKE_VALID_PIN)) == static_cast<bool>(BrakeStatus::VALID) && ThrottleBrake::brake_opened_or_shorted_timer.GetTimerState() == VirtualTimer::State::kRunning) {
        ThrottleBrake::brake_opened_or_shorted_timer.Disable();
        ThrottleBrake::brake_opened_or_shorted_timer.Enable();
    }
    // if implausibility detected and timer already running, then do nothing (timer ticks in main)
    // if there is no implausibility detected and no current implausibility timer running, then do nothing

};

/**
 * @brief Returns true if APPS1 and APPS2 disagree by >10% for >100ms, 
 *        Returns False otherwise
 *        (T.4.2.4 - T.4.2.5)
 *
 * @return void
 */
void ThrottleBrake::check_APPSs_differing_by_10_percent_implausibility() {
    // Convert APPS raw values to throttle percentages (0-100)
    float APPS1_percentage = ((ThrottleBrake::APPS1_raw - static_cast<int32_t>(Bounds::APPS1_RAW_MIN)) * 100.0) / static_cast<int32_t>(Bounds::APPS1_RAW_SPAN);
    float APPS2_percentage = ((static_cast<int32_t>(Bounds::APPS2_RAW_MAX) - ThrottleBrake::APPS2_raw) * 100.0) / static_cast<int32_t>(Bounds::APPS2_RAW_SPAN);

    int32_t APPS_diff = APPS1_percentage - APPS2_percentage; // Get percentage point difference between APPS values
    // if (APPS_diff > 10.0 || APPS_diff < -10.0) { // If values differ by more than 10 percentage points
    //     if (ThrottleBrake::APPSs_differ_by_10_percent_timer.GetTimerState() == VirtualTimer::State::kNotStarted) {
    //         ThrottleBrake::APPSs_differ_by_10_percent_timer.Start(millis());
    //     }
    // } else if (ThrottleBrake::APPSs_differ_by_10_percent_timer.GetTimerState() == VirtualTimer::State::kRunning) {
    //     ThrottleBrake::APPSs_differ_by_10_percent_timer.Disable();
    //     ThrottleBrake::APPSs_differ_by_10_percent_timer.Enable();
    // }
    if ((APPS_diff > 10.0 || APPS_diff < -10.0) && ThrottleBrake::APPSs_differ_by_10_percent_timer.GetTimerState() == VirtualTimer::State::kNotStarted) {
        ThrottleBrake::APPSs_differ_by_10_percent_timer.Start(millis());
        
    } else if (!(APPS_diff > 10.0 || APPS_diff < -10.0) && ThrottleBrake::APPSs_differ_by_10_percent_timer.GetTimerState() == VirtualTimer::State::kRunning) {
        ThrottleBrake::APPSs_differ_by_10_percent_timer.Disable();
        ThrottleBrake::APPSs_differ_by_10_percent_timer.Enable();
    }
    // if implausibility detected and timer already running, then do nothing (timer ticks in main)
    // if there is no implausibility detected and no current implausibility timer running, then do nothing


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
    if (ThrottleBrake::front_brake_raw > static_cast<int32_t>(Bounds::FRONT_BRAKE_RAW_PRESSED_THRESHOLD)) {
        ThrottleBrake::brake_pressed = true;
        return true;
    }
    return false;
};

/**
 * @brief Returns true if both brake is pressed and throttle is >25%, 
 *        Returns false after throttle returns to <=5% (regardless of brake) or otherwise 
 *        (EV.4.7)
 *
 * @return 
 */
void ThrottleBrake::check_BPPC_implausibility() {
    float APPS1_percentage = ((ThrottleBrake::APPS1_raw - static_cast<int32_t>(Bounds::APPS1_RAW_MIN)) * 100.0) / static_cast<int32_t>(Bounds::APPS1_RAW_SPAN);

    if (ThrottleBrake::is_brake_pressed() && (APPS1_percentage > 25.0)) {
        ThrottleBrake::BPPC_implausibility_present = true;
    }

    if (APPS1_percentage < 5.0) {
        ThrottleBrake::BPPC_implausibility_present = false;
    }
};

/**
 * @brief Check for implausibility and set throttle/brake CAN signals accordingly
 *
 * @return void
 */
void ThrottleBrake::update_throttle_brake_CAN_signals() {
    ThrottleBrake::throttle_percent = ThrottleBrake::APPS1;
    ThrottleBrake::front_brake_pressure = ThrottleBrake::front_brake;
    ThrottleBrake::rear_brake_pressure = ThrottleBrake::rear_brake;
    ThrottleBrake::brake_pressed_signal = ThrottleBrake::brake_pressed;
    ThrottleBrake::implausibility_present_signal = ThrottleBrake::is_implausibility_present();
};
