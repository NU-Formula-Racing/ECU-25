#include <Arduino.h>

#include "throttle_brake_driver.hpp"
#include "pins.hpp"

/**
 * @brief Initializes pins (uses pinMode() to set up pins)
 *
 * @return 
 */

void ThrottleBrake::initialize() {
    pinMode((uint8_t)Pins::APPS1_CS_PIN, INPUT);
    pinMode((uint8_t)Pins::APPS2_CS_PIN, INPUT);
    pinMode((uint8_t)Pins::FRONT_BRAKE_CS_PIN, INPUT);
    pinMode((uint8_t)Pins::REAR_BRAKE_CS_PIN, INPUT);
    pinMode((uint8_t)Pins::BRAKE_VALID_PIN, INPUT);
    pinMode((uint8_t)Pins::READY_TO_DRIVE_SWITCH, INPUT);
    pinMode((uint8_t)Pins::TS_ACTIVE_PIN, INPUT);
};

/**
 * @brief Reads data from ADCs and stores RAW sensor data (in ADC counts) in class variables  
 */
void ThrottleBrake::read_ADCs() {
    
};

/**
 * @brief Gets APPS1 value SCALED from 0-32767
 *
 * @return uint16_t
 */   
uint16_t ThrottleBrake::get_APPS1() {
    return (ThrottleBrake::APPS1_raw - (uint16_t)Bounds::APPS1_MIN) * 32767 / (uint16_t)Bounds::APPS1_RANGE;
};
        
/**
 * @brief Gets APPS2 value SCALED from 0-32767
 *
 * @return uint16_t
 */
uint16_t ThrottleBrake::get_APPS2() {
    return ((uint16_t)Bounds::APPS2_MAX - ThrottleBrake::APPS2_raw) * 32767 / (uint16_t)Bounds::APPS2_RANGE;
};
        
/**
 * @brief Gets Front Brake value SCALED from 0-32767
 *
 * @return uint16_t
 */
uint16_t ThrottleBrake::get_front_brake() {
    return (ThrottleBrake::front_brake_raw - (uint16_t)Bounds::FRONT_BRAKE_MIN) * 32767 / (uint16_t)Bounds::FRONT_BRAKE_RANGE;
}
        
/**
 * @brief Gets Rear Brake value SCALED from 0-32767
 *
 * @return unt16_t
 */ 
uint16_t ThrottleBrake::get_rear_brake() {
    return (ThrottleBrake::rear_brake_raw - (uint16_t)Bounds::REAR_BRAKE_MIN) * 32767 / (uint16_t)Bounds::REAR_BRAKE_RANGE;
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
    if (ThrottleBrake::front_brake > (uint16_t)Bounds::BRAKE_PRESSED_THRESHOLD) {
        brake_pressed_signal = true;
        return true;
    }
    brake_pressed_signal = false;
    return false;
};
        
/**
 * @brief Returns true if any implausibility is present, false otherwise
 *
 * @return bool
 */
bool ThrottleBrake::is_implausibility_present() {
    // if front brake or APPS fail any implausibility checks, return true
    return (ThrottleBrake::is_brake_implausible() || ThrottleBrake::is_10_percent_rule_implausible() || ThrottleBrake::is_BPPC_implausible());
};
        
/**
 * @brief Returns true if brake is implausible (open or shorted for >100ms), 
 *        Returns false otherwise
 *        (T.4.3.3)
 *
 * @return bool
 */     
bool ThrottleBrake::is_brake_implausible() {
    return (ThrottleBrake::front_brake_raw > (uint16_t)Bounds::BRAKE_DEFAULT_HIGH_INPUT_RAW_ADC_THRESHOLD || ThrottleBrake::front_brake_raw < (uint16_t)Bounds::BRAKE_DEFAULT_LOW_INPUT_RAW_ADC_THRESHOLD);
};

/**
 * @brief Returns true if APPS1 and APPS2 disagree by >10% for >100ms, 
 *        Returns False otherwise
 *        (T.4.2.4 - T.4.2.5)
 *
 * @return void
 */
bool ThrottleBrake::is_10_percent_rule_implausible() {
    // Convert APPS values from uint16_t (0-32767) to float (0-100)
    float APPS1_percentage = (ThrottleBrake::APPS1_raw - (uint16_t)Bounds::APPS1_MIN) / (uint16_t)Bounds::APPS1_RANGE * 100.0;
    throttle_percent = APPS1_percentage;
    float APPS2_percentage = ((uint16_t)Bounds::APPS2_MAX - ThrottleBrake::APPS2_raw) / (uint16_t)Bounds::APPS2_RANGE * 100.0;
    int16_t APPS_diff = APPS1_percentage - APPS2_percentage; // Get percentage point difference between APPS values
    if (APPS_diff > 10.0 || APPS_diff < -10.0) { // If values differ by more than 10 percentage points
        long current_time = millis();
        if (ThrottleBrake::time_of_start_of_ten_percent_implasibility != 0) { // If this isn't the start of an implausibility
            if (current_time - ThrottleBrake::time_of_start_of_ten_percent_implasibility < 100) { // If it's been more than 100 milliseconds since the implausibility started
                return true;
            }
        } else { // This is the start of an implausibility
            ThrottleBrake::time_of_start_of_ten_percent_implasibility = current_time; // Record the start time
        }
    } else {
        ThrottleBrake::time_of_start_of_ten_percent_implasibility = 0; // No implausibility detected, set start time to zero
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
bool ThrottleBrake::is_BPPC_implausible() {
    if (ThrottleBrake::front_brake > (uint16_t)Bounds::FRONT_BRAKE_MIN && ThrottleBrake::rear_brake > (uint16_t)Bounds::REAR_BRAKE_MIN && ThrottleBrake::APPS1 > 8191) {
        if (!ThrottleBrake::throttle_dropping_to_5_percent_after_brake_implausibility) {
            ThrottleBrake::throttle_dropping_to_5_percent_after_brake_implausibility = true;
        }
        return true;
    } else if (ThrottleBrake::throttle_dropping_to_5_percent_after_brake_implausibility && ThrottleBrake::APPS1 <= 1638) {
        ThrottleBrake::throttle_dropping_to_5_percent_after_brake_implausibility = false;
    }
    return false;
};

/**
 * @brief Check for implausibility and set throttle/brake CAN signals accordingly
 *
 * @return void
 */
void ThrottleBrake::send_throttle_brake_CAN() {

};
