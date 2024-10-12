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
 * @return int16_t
 */   
int16_t get_APPS1() {

    int16_t APPS1 = max(analogRead((int)Pins::APPS1_CS_PIN);
    float APPS1_throttl

}; 
        
/**
 * @brief Gets APPS2 value SCALED from 0-32767
 *
 * @return int16_t
 */
int16_t get_APPS2() {

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