#include <Arduino.h>

#include "throttle_brake_driver.hpp"
#include "pins.hpp"

/**
 * @brief Initializes pins
 *
 * @return 
 */
void ThrottleBrake::initialize() {
    // use pinMode() to set up pins
};

/**
 * @brief Reads data from ADCs and stores RAW sensor data (in ADC counts) in class variables  
 */
void ThrottleBrake::read_ADCs() {

};
        
 /**
 * @brief Gets APPS1 value SCALED from 0-32767
 *
 * @return int16_t
 */   
int16_t ThrottleBrake::get_APPS1() {

}; 
        
/**
 * @brief Gets APPS2 value SCALED from 0-32767
 *
 * @return int16_t
 */
int16_t ThrottleBrake::get_APPS2() {

};
        
/**
 * @brief Gets Front Brake value SCALED from 0-32767
 *
 * @return int16_t
 */
int16_t ThrottleBrake::get_front_brake() {

};
        
/**
 * @brief Gets Rear Brake value SCALED from 0-32767
 *
 * @return int16_t
 */ 
int16_t ThrottleBrake::get_rear_brake() {

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
};
        
/**
 * @brief Returns true if any implausibility is present, false otherwise
 *
 * @return bool
 */
bool ThrottleBrake::is_implausibility_present() {
    // if front brake or APPS fail any implausibility checks, return true

};
        
/**
 * @brief Returns true if brake is implausible (open or shorted for >100ms), 
 *        Returns false otherwise
 *        (T.4.3.3)
 *
 * @return bool
 */     
bool ThrottleBrake::is_brake_implausible() {

};

/**
 * @brief Returns true if APPS1 and APPS2 disagree by >10% for >100ms, 
 *        Returns False otherwise
 *        (T.4.2.4 - T.4.2.5)
 *
 * @return void
 */
bool ThrottleBrake::is_10_percent_rule_implausible() {

};

/**
 * @brief Returns true if both brake is pressed and throttle is >25%, 
 *        Returns false after throttle returns to >=5% (regardless of brake) or otherwise 
 *        (EV.4.7)
 *
 * @return 
 */
bool ThrottleBrake::is_BPPC_implausible() {

};

/**
 * @brief Check for implausibility and set throttle/brake CAN signals accordingly
 *
 * @return void
 */
void ThrottleBrake::send_throttle_brake_CAN() {

};