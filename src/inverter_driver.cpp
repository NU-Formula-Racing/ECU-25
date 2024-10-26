#include <Arduino.h>

#include "inverter_driver.hpp"
#include "pins.hpp"

/**
 * @brief 
 *
 * @return 
 */
void Inverter::initialize() {

}

/**
 * @brief 
 *
 * @return 
 */
void Inverter::get_motor_rpm() {

}

/**
 * @brief
 * 
 * @return
 */
void Inverter::get_IGBT_temp() {

}

/**
 * @brief
 * 
 * @return
*/
void Inverter::get_motor_temp() {

}

/**
 * @brief
 * 
 * @return
 */
void Inverter::read_inverter_CAN() {
    motor_rpm = ERPM;
    IGBT_temp = Temp_FET;
    motor_temp = Temp_Motor;
}

/**
 * @brief
 * 
 * @return
 */
void Inverter::send_inverter_CAN() {
    // Set_Current = ;
    // Set_Current_Brake = ;
}

/**
 * @brief
 * @param torque_mA -- torque in milliAmps
 * @return
 */
void Inverter::request_torque(int32_t torque_mA) {

}