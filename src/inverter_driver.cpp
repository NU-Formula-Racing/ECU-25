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
  Inverter::IGBT_temp = 150;
  Inverter::motor_temp = 120;
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
 * @brief Get set current
 *
 * @return int32_t
 */
int32_t Inverter::get_set_current() const { return Inverter::requested_torque_throttle; }
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
 * @param torque_reqs -- <accel, regen> torque in milliAmps
 * @return void
 */
void Inverter::request_torque(std::pair<int32_t, int32_t> torque_reqs) {
  Inverter::requested_torque_throttle = torque_reqs.first;
  Inverter::requested_torque_brake = torque_reqs.second;
}

void Inverter::print_inverter_info() {
  Serial.print(" Set_Cur: ");
  Serial.print(Inverter::requested_torque_throttle);
  Serial.print(" Set_Cur_Br: ");
  Serial.print(Inverter::requested_torque_brake);

  // Serial.print(" RPM: ");
  // Serial.print(Inverter::motor_rpm);
  // Serial.print(" IGBT_temp: ");
  // Serial.print(Inverter::IGBT_temp);
  // Serial.print(" Motor_temp: ");
  // Serial.print(Inverter::motor_temp);
}