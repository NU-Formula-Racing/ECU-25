#include <Arduino.h>

#include "inverter_driver.hpp"
#include "pins.hpp"

/**
 * @brief Initialize Inverter class
 *
 * @return void
 */
void Inverter::initialize() {
    requested_torque_brake = 0;
    requested_torque_throttle = 0;
}

/**
 * @brief Get RPM of motor
 *
 * @return int32_t
 */
int32_t Inverter::get_motor_rpm() {
    return motor_rpm;
}

/**
 * @brief Get IGBT temperature
 * 
 * @return int16_t
 */
int16_t Inverter::get_IGBT_temp() {
    return IGBT_temp;
}

/**
 * @brief Get motor temperature
 * 
 * @return int16_t
*/
int16_t Inverter::get_motor_temp() {
    return motor_temp;
}

/**
 * @brief Read CAN messages from Inverter and set class variables accordingly
 * 
 * @return void
 */
void Inverter::read_inverter_CAN() {
    motor_rpm = ERPM;
    IGBT_temp = Temp_FET;
    motor_temp = Temp_Motor;
}

/**
 * @brief Send CAN messages to Inverter (set current and set current brake)
 * 
 * @return void
 */
void Inverter::send_inverter_CAN() {
    set_current = requested_torque_throttle;
    set_current_brake = requested_torque_brake;
}

/**
 * @brief Request torque from Inverter
 * @param torque_mA -- torque in milliAmps
 * @return void
 */
void Inverter::request_torque(int32_t torque_mA) {
    if (torque_mA >= 0) {
        requested_torque_throttle = torque_mA;
        requested_torque_brake = 0;
    } else {
        requested_torque_brake = torque_mA;
        requested_torque_throttle = 0;
    }
}