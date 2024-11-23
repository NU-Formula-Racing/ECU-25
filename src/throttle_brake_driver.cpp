#ifdef ARDUINO
#include <Arduino.h>
#include <SPI.h>
#endif

#include "throttle_brake_driver.hpp"
#include "pins.hpp"


void ThrottleBrake::initialize_CS_pin(uint8_t CS_pin) {
    pinMode(CS_pin, OUTPUT);
    digitalWrite(CS_pin, HIGH);
}

void ThrottleBrake::initialize_CS_pins(std::vector<uint8_t> CS_pins) {
    for (uint8_t CS_pin : CS_pins) ThrottleBrake::initialize_CS_pin(CS_pin);
}

/**
 * @brief Initializes pins (uses pinMode() to set up pins)
 */
void ThrottleBrake::initialize() {

    SPI.begin();

    // for all of these: instead of using (uint8_t)Pins::PIN_NAME, use static_cast<uint8_t>(Pins::PIN_NAME)
    // static_cast is the C++ style of casting, and it's safer and more explicit than the C style casting (uint8_t)

    std::vector<uint8_t> CS_pins = {
        static_cast<uint8_t>(Pins::APPS1_CS_PIN),
        static_cast<uint8_t>(Pins::APPS2_CS_PIN),
        static_cast<uint8_t>(Pins::FRONT_BRAKE_CS_PIN),
        static_cast<uint8_t>(Pins::REAR_BRAKE_CS_PIN)
    };
    ThrottleBrake::initialize_CS_pins(CS_pins);

    pinMode(static_cast<uint8_t>(Pins::BRAKE_VALID_PIN), INPUT);

};

/**
 * @brief Reads data from ADCs and stores RAW sensor data (in ADC counts) in class variables  
 */
void ThrottleBrake::readADCs() {

    // 1000000: clock rate
    // MSBFIRST: Most significant bit first
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));


    // fix casting


    // read APPS1
    digitalWrite(static_cast<uint8_t>(Pins::APPS1_CS_PIN), LOW);
    ThrottleBrake::APPS1_raw = (SPI.transfer32(0x0000) << 2) >> 4; // need to shift left 2 to shift off the leading 2 0's, then shift right 4 to sign-extend
    digitalWrite(static_cast<uint8_t>(Pins::APPS1_CS_PIN), HIGH);

    // read APPS2
    digitalWrite((uint8_t)Pins::APPS2_CS_PIN, LOW); // fix casting
    ThrottleBrake::APPS2_raw = SPI.transfer32(0x0000) >> 2; // need to shift left 2 to shift off the leading 2 0's, then shift right 4 to sign-extend
    digitalWrite((uint8_t)Pins::APPS2_CS_PIN, HIGH); // fix casting

    // read front_brake
    digitalWrite((uint8_t)Pins::FRONT_BRAKE_CS_PIN, LOW); // fix casting
    ThrottleBrake::front_brake_raw= SPI.transfer32(0x0000) >> 2; // need to shift left 2 to shift off the leading 2 0's, then shift right 4 to sign-extend
    digitalWrite((uint8_t)Pins::FRONT_BRAKE_CS_PIN, HIGH); // fix casting

    // read rear_brake
    digitalWrite((uint8_t)Pins::REAR_BRAKE_CS_PIN, LOW); // fix casting
    ThrottleBrake::rear_brake_raw = SPI.transfer32(0x0000) >> 2; // need to shift left 2 to shift off the leading 2 0's, then shift right 4 to sign-extend
    digitalWrite((uint8_t)Pins::REAR_BRAKE_CS_PIN, HIGH); // fix casting

    SPI.endTransaction();

};

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
 * @brief Returns true if brake is pressed, false otherwise
 *
 * @return bool
 */
bool ThrottleBrake::is_brake_pressed() {
    // check if the front brake value is over a certain threshold (actual threshold is TBD, need to test with brake sensors)
    // if yes: return true
    // if no: return false
    if (ThrottleBrake::front_brake > static_cast<int32_t>(Bounds::BRAKE_PRECSED_THRESHOLD) { // fix casting
        brake_pressed_signal = true; // this is a CAN signal, we only want to be changing the CAN signals in the send_throttle_brake_CAN() function. use brake_pressed instead
        return true;
    }
    brake_pressed_signal = false; // this is a CAN signal, we only want to be changing the CAN signals in the send_throttle_brake_CAN() function. use brake_pressed instead
    return false;
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
    // this doesnt account for the rules-requires 100ms delay, use a timer for this
    return (ThrottleBrake::front_brake_raw > static_cast<int32_t>(Bounds::FRONT_BRAKE_RAW_DEFAULT_HIGH_THRESHOLD) || ThrottleBrake::front_brake_raw < static_cast<int32_t>(Bounds::FRONT_BRAKE_RAW_DEFAULT_LOW_THRESHOLD));
};

/**
 * @brief Returns true if APPS1 and APPS2 disagree by >10% for >100ms, 
 *        Returns False otherwise
 *        (T.4.2.4 - T.4.2.5)
 *
 * @return void
 */
bool ThrottleBrake::is_10_percent_rule_implausible() {
    // Convert APPS values from int32_t (0-32767) to float (0-100)
    float APPS1_percentage = ((ThrottleBrake::APPS1_raw - static_cast<int32_t>(Bounds::APPS1_RAW_MIN)) * 100.0) / static_cast<int32_t>(Bounds::APPS1_RAW_SPAN);
    float APPS2_percentage = ((static_cast<int32_t>(Bounds::APPS2_RAW_MAX) - ThrottleBrake::APPS2_raw) * 100.0) / static_cast<int32_t>(Bounds::APPS2_RAW_SPAN);

    int32_t APPS_diff = APPS1_percentage - APPS2_percentage; // Get percentage point difference between APPS values
    if (APPS_diff > 10.0 || APPS_diff < -10.0) { // If values differ by more than 10 percentage points
        
    } else {

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
    if (ThrottleBrake::front_brake > static_cast<int32_t>(Bounds::FRONT_BRAKE_RAW_MIN && ThrottleBrake::rear_brake > static_cast<int32_t>(Bounds::REAR_BRAKE_RAW_MIN && ThrottleBrake::APPS1 > 8191) {
        if (!ThrottleBrake::throttle_dropping_to_5_percent_after_brake_implausibility) {
            ThrottleBrake::throttle_dropping_to_5_percent_after_brake_implausibility = true;
        }
        return true;
    } else if (ThrottleBrake::throttle_dropping_to_5_percent_after_brake_implausibility && ThrottleBrake::APPS1 <= 3238) {
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
