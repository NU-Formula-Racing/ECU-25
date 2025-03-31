#include "inverter_driver.hpp"

#include <Arduino.h>

#include "pins.hpp"
#include "throttle_brake_driver.hpp"

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
int32_t Inverter::get_motor_rpm() const { return Inverter::motor_rpm; }

/**
 * @brief Get IGBT temperature
 *
 * @return int16_t
 */
int16_t Inverter::get_IGBT_temp() const { return Inverter::IGBT_temp; }

/**
 * @brief Get motor temperature
 *
 * @return int16_t
 */
int16_t Inverter::get_motor_temp() const { return Inverter::motor_temp; }

/**
 * @brief Read CAN messages from Inverter and set class variables accordingly
 *
 * @return void
 */
void Inverter::read_inverter_CAN() {
  Inverter::motor_rpm = Inverter::RPM;
  Inverter::IGBT_temp = Inverter::IGBT_Temp;
  Inverter::motor_temp = Inverter::Motor_Temp;
}

/**
 * @brief Send CAN messages to Inverter (set_current and set_current_brake)
 *
 * @return void
 */
void Inverter::send_inverter_CAN() {
  Inverter::Set_Current = Inverter::requested_torque_throttle;
  Inverter::Set_Current_Brake = Inverter::requested_torque_brake;
}

/**
 * @brief Request torque from Inverter
 * @param torque_mA -- torque in milliAmps
 * @return void
 */
void Inverter::request_torque(int32_t torque_mA) {
  if (throttle_brake.is_brake_pressed()) {
    Inverter::requested_torque_throttle = 0;
    Inverter::requested_torque_brake = torque_mA;
  } else {
    Inverter::requested_torque_throttle = torque_mA;
    Inverter::requested_torque_brake = 0;
  }
}

void Inverter::print_inverter_info() {
  Serial.print(" Set_Current: ");
  Serial.print(Inverter::requested_torque_throttle);
  Serial.print(" Set_Current_Brake: ");
  Serial.print(Inverter::requested_torque_brake);
}