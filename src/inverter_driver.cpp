#ifdef ARDUINO
#include <Arduino.h>
#endif

#include "inverter_driver.hpp"
#include "pins.hpp"

/**
 * @brief Initialize Inverter class
 *
 * @return void
 */
void Inverter::initialize() {
    Inverter::requested_torque_brake = 0;
    Inverter::requested_torque_throttle = 0;
}

/**
 * @brief Get RPM of motor
 *
 * @return int32_t
 */
int32_t Inverter::get_motor_rpm() {
    return Inverter::motor_rpm;
}

/**
 * @brief Get IGBT temperature
 * 
 * @return int16_t
 */
int16_t Inverter::get_IGBT_temp() {
    return Inverter::IGBT_temp;
}

/**
 * @brief Get motor temperature
 * 
 * @return int16_t
*/
int16_t Inverter::get_motor_temp() {
    return Inverter::motor_temp;
}

/**
 * @brief Read CAN messages from Inverter and set class variables accordingly
 * 
 * @return void
 */
void Inverter::read_inverter_CAN() {
    Inverter::motor_rpm = Inverter::ERPM; 
    Inverter::IGBT_temp = Inverter::Temp_FET;
    Inverter::motor_temp = Inverter::Temp_Motor;
}

/**
 * @brief Send CAN messages to Inverter (set_current and set_current_brake)
 * 
 * @return void
 */
void Inverter::send_inverter_CAN() {
    Inverter::set_current = Inverter::requested_torque_throttle;
    Inverter::set_current_brake = Inverter::requested_torque_brake;
}

/**
 * @brief Request torque from Inverter
 * @param torque_mA -- torque in milliAmps
 * @return void
 */
void Inverter::request_torque(int32_t torque_mA) {
    if (torque_mA >= 0) {
        Inverter::requested_torque_throttle = torque_mA;
        Inverter::requested_torque_brake = 0;
    } else {
        Inverter::requested_torque_brake = torque_mA;
        Inverter::requested_torque_throttle = 0;
    }
}