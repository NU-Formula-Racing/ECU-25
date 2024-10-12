#include <Arduino.h>

#include "throttle_brake_driver.hpp"
#include "pins.hpp"

/**
 * @brief Initializes 
 *
 * @return 
 */
void ThrottleBrake::initialize() {

};

/**
 * @brief Reads data from ADCs and stores RAW sensor data (in ADC counts) in class variables  
 */
void read_ADCs() {

};
        
 /**
 * @brief Gets APPS1 value SCALED from 0-32767
 *
 * @return uint16_t
 */   
int16_t ThrottleBrake::get_APPS1() {
    uint16_t APPS1_raw = analogRead((uint8_t)Pins::APPS1_CS_PIN);
    ThrottleBrake:APPS1_value = APPS1_raw;
    if (APPS1_raw > (uint16_t)Bounds::APPS1_MAX || APPS1_raw < (uint16_t)Bounds::APPS1_MIN) return 0;
    return (APPS1_raw - (uint16_t)Bounds::APPS1_MIN) * 32767 / (uint16_t)Bounds::APPS1_RANGE;
};
        
/**
 * @brief Gets APPS2 value SCALED from 0-32767
 *
 * @return uint16_t
 */
uint16_t get_APPS2() {
    uint16_t APPS2_raw = analogRead((uint8_t)Pins::APPS2_CS_PIN);
    if (APPS2_raw > (uint16_t)Bounds::APPS2_MAX || APPS2_raw < (uint16_t)Bounds::APPS2_MIN) return 0;
    return ((uint16_t)Bounds::APPS2_MAX - APPS2_raw) * 32767 / (uint16_t)Bounds::APPS2_RANGE;
};
        
/**
 * @brief 
 *
 * @return 
 */
int16_t get_front_brake() {

};
        
/**
 * @brief 
 *
 * @return 
 */ 
int16_t get_rear_brake() {

};
        
/**
 * @brief 
 *
 * @return 
 */
bool is_brake_pressed() {

};
        
/**
 * @brief 
 *
 * @return 
 */
bool is_implausibility_present() {

};
        
/**
 * @brief 
 *
 * @return 
 */     
bool is_brake_valid() {

};

/**
 * @brief 
 *
 * @return 
 */
void check_ten_percent_rule() {
    if (get_APPS1() / get_APPS2() > 1.1)
};

/**
 * @brief 
 *
 * @return 
 */
void check_BPPC_rule() {

};