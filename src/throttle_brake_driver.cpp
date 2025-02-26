#include "throttle_brake_driver.hpp"

#include <Arduino.h>
#include <SPI.h>

#include "pins.hpp"

void ThrottleBrake::initialize_CS_pin(uint8_t CS_pin) {
  pinMode(CS_pin, OUTPUT);
  digitalWrite(CS_pin, HIGH);
}

void ThrottleBrake::initialize_CS_pins() {
  initialize_CS_pin(static_cast<uint8_t>(Pins::APPS1_CS_PIN));
  initialize_CS_pin(static_cast<uint8_t>(Pins::APPS2_CS_PIN));
  initialize_CS_pin(static_cast<uint8_t>(Pins::FRONT_BRAKE_CS_PIN));
  initialize_CS_pin(static_cast<uint8_t>(Pins::REAR_BRAKE_CS_PIN));
}

/**
 * @brief Set implausibilities false, start SPI bus, set pin modes, write default HIGH to CS pins
 */
void ThrottleBrake::initialize() {
  ThrottleBrake::APPSs_disagreement_implausibility_present = false;
  ThrottleBrake::BPPC_implausibility_present = false;
  ThrottleBrake::brake_shorted_or_opened_implausibility_present = false;

  SPI.begin(static_cast<uint8_t>(Pins::SPI_CLK), static_cast<uint8_t>(Pins::SPI_MISO),
            static_cast<uint8_t>(Pins::SPI_MOSI));

  initialize_CS_pins();

  pinMode(static_cast<uint8_t>(Pins::BRAKE_VALID_PIN), INPUT);
};

/**
 * @brief Reads data from ADCs and stores adc sensor data (in ADC counts) in class variables
 */
void ThrottleBrake::read_from_SPI_ADCs() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
  // 1000000: clock rate
  // MSBFIRST: Most significant bit first

  digitalWrite(static_cast<uint8_t>(Pins::APPS1_CS_PIN), LOW);

  int16_t APPS1 = SPI.transfer16(0x0000);
  ThrottleBrake::APPS1_adc = (APPS1 << 2) >> 4;
  digitalWrite(static_cast<uint8_t>(Pins::APPS1_CS_PIN), HIGH);

  digitalWrite(static_cast<uint8_t>(Pins::APPS2_CS_PIN), LOW);
  int16_t APPS2 = SPI.transfer16(0x0000);
  ThrottleBrake::APPS2_adc = (APPS2 << 2) >> 4;
  digitalWrite(static_cast<uint8_t>(Pins::APPS2_CS_PIN), HIGH);

  digitalWrite(static_cast<uint8_t>(Pins::FRONT_BRAKE_CS_PIN), LOW);
  int16_t front = SPI.transfer16(0x0000);
  ThrottleBrake::front_brake_adc = (front << 2) >> 4;

  digitalWrite(static_cast<uint8_t>(Pins::FRONT_BRAKE_CS_PIN), HIGH);

  digitalWrite(static_cast<uint8_t>(Pins::REAR_BRAKE_CS_PIN), LOW);
  int16_t rear = SPI.transfer16(0x0000);
  ThrottleBrake::rear_brake_adc = (rear << 2) >> 4;
  digitalWrite(static_cast<uint8_t>(Pins::REAR_BRAKE_CS_PIN), HIGH);

  SPI.endTransaction();
}

int16_t ThrottleBrake::get_safe_RAW(int16_t RAW, int16_t projected_min, int16_t projected_max) {
  if (RAW < projected_min) {
    return projected_min;
  } else if (RAW > projected_max) {
    return projected_max;
  } else {
    return RAW;
  }
}

void ThrottleBrake::update_sensor_values() {
  ThrottleBrake::read_from_SPI_ADCs();

  int16_t safe_APPS1_adc =
      get_safe_RAW(ThrottleBrake::APPS1_adc, static_cast<int16_t>(Bounds::APPS1_ADC_MIN),
                   static_cast<int16_t>(Bounds::APPS1_ADC_MAX));
  ThrottleBrake::APPS1_throttle_scaled =
      ((safe_APPS1_adc - static_cast<int32_t>(Bounds::APPS1_ADC_MIN)) * 32767) /
      static_cast<int32_t>(Bounds::APPS1_ADC_SPAN);

  int16_t safe_APPS2_ADC =
      get_safe_RAW(ThrottleBrake::APPS2_adc, static_cast<int16_t>(Bounds::APPS2_ADC_MIN),
                   static_cast<int16_t>(Bounds::APPS2_ADC_MAX));
  ThrottleBrake::APPS2_throttle_scaled =
      ((static_cast<int32_t>(Bounds::APPS2_ADC_MAX) - safe_APPS2_ADC) * 32767) /
      static_cast<int32_t>(Bounds::APPS2_ADC_SPAN);

  int16_t safe_front_brake_adc = get_safe_RAW(ThrottleBrake::front_brake_adc,
                                              static_cast<int16_t>(Bounds::FRONT_BRAKE_ADC_MIN),
                                              static_cast<int16_t>(Bounds::FRONT_BRAKE_ADC_MAX));
  ThrottleBrake::front_brake_scaled =
      ((safe_front_brake_adc - static_cast<int16_t>(Bounds::FRONT_BRAKE_ADC_MIN)) * 32767) /
      static_cast<int16_t>(Bounds::FRONT_BRAKE_ADC_SPAN);

  int16_t safe_rear_brake_adc =
      get_safe_RAW(ThrottleBrake::rear_brake_adc, static_cast<int16_t>(Bounds::REAR_BRAKE_ADC_MIN),
                   static_cast<int16_t>(Bounds::REAR_BRAKE_ADC_MAX));
  ThrottleBrake::rear_brake_scaled =
      (safe_rear_brake_adc - static_cast<int16_t>(Bounds::REAR_BRAKE_ADC_MIN) * 32767) /
      static_cast<int16_t>(Bounds::REAR_BRAKE_ADC_SPAN);

  if (ThrottleBrake::front_brake_adc >=
      static_cast<int16_t>(Bounds::FRONT_BRAKE_ADC_PRESSED_THRESHOLD)) {
    ThrottleBrake::brake_pressed = true;
  } else {
    ThrottleBrake::brake_pressed = false;
  }
}

/**
 * @brief Returns APPS1 throttle value, scaled 0-32767
 *
 * @return int16_t
 */
int16_t ThrottleBrake::get_throttle() { return ThrottleBrake::APPS1_throttle_scaled; };

/**
 * @brief Returns true if any implausibility is present, false otherwise
 *
 * @return bool
 */
bool ThrottleBrake::is_implausibility_present() {
  return ThrottleBrake::APPSs_disagreement_implausibility_present ||
         ThrottleBrake::brake_shorted_or_opened_implausibility_present ||
         ThrottleBrake::BPPC_implausibility_present;
};

void ThrottleBrake::set_is_APPSs_disagreement_implausibility_present_to_true() {
  ThrottleBrake::APPSs_disagreement_implausibility_present = true;
};

void ThrottleBrake::set_is_brake_shorted_or_opened_implausibility_present_to_true() {
  ThrottleBrake::brake_shorted_or_opened_implausibility_present = true;
};

void ThrottleBrake::check_for_implausibilities() {
  ThrottleBrake::check_APPSs_disagreement_implausibility();
  ThrottleBrake::check_BPPC_implausibility();
  ThrottleBrake::check_brake_shorted_or_opened_implausibility();
}

/**
 * @brief Checks if brake is implausible (open or shorted for >100ms). (T.4.3.3)
 *
 * @return void
 */
void ThrottleBrake::check_brake_shorted_or_opened_implausibility() {
  if (digitalRead(static_cast<uint8_t>(Pins::BRAKE_VALID_PIN)) ==
          static_cast<bool>(BrakeStatus::INVALID) &&
      ThrottleBrake::brake_shorted_or_opened_implausibility_timer.GetTimerState() ==
          VirtualTimer::State::kNotStarted) {
    ThrottleBrake::brake_shorted_or_opened_implausibility_timer.Start(millis());
  } else if (digitalRead(static_cast<uint8_t>(Pins::BRAKE_VALID_PIN)) ==
                 static_cast<bool>(BrakeStatus::VALID) &&
             ThrottleBrake::brake_shorted_or_opened_implausibility_timer.GetTimerState() ==
                 VirtualTimer::State::kRunning) {
    ThrottleBrake::brake_shorted_or_opened_implausibility_timer.Disable();
    ThrottleBrake::brake_shorted_or_opened_implausibility_timer.Enable();
  }
  // if implausibility detected and timer already running, then do nothing (timer ticks in main)
  // if there is no implausibility detected and no current implausibility timer running, then do
  // nothing
};

/**
 * @brief Returns true if APPS1 and APPS2 disagree by >10% for >100ms,
 *        Returns False otherwise
 *        (T.4.2.4 - T.4.2.5)
 *
 * @return void
 */
void ThrottleBrake::check_APPSs_disagreement_implausibility() {
  // Convert APPS raw adc values to throttle percentages (0-100)
  float APPS1_percentage =
      ((ThrottleBrake::APPS1_adc - static_cast<int32_t>(Bounds::APPS1_ADC_MIN)) * 100.0) /
      static_cast<int32_t>(Bounds::APPS1_ADC_SPAN);
  float APPS2_percentage =
      ((static_cast<int32_t>(Bounds::APPS2_ADC_MAX) - ThrottleBrake::APPS2_adc) * 100.0) /
      static_cast<int32_t>(Bounds::APPS2_ADC_SPAN);
  int32_t APPS_diff =
      APPS1_percentage - APPS2_percentage;  // Get percentage point difference between APPS values
  if ((APPS_diff > 10.0 || APPS_diff < -10.0) &&
      ThrottleBrake::APPSs_disagreement_implausibility_timer.GetTimerState() ==
          VirtualTimer::State::kNotStarted) {
    ThrottleBrake::APPSs_disagreement_implausibility_timer.Start(millis());

  } else if (!(APPS_diff > 10.0 || APPS_diff < -10.0) &&
             ThrottleBrake::APPSs_disagreement_implausibility_timer.GetTimerState() ==
                 VirtualTimer::State::kRunning) {
    ThrottleBrake::APPSs_disagreement_implausibility_timer.Disable();
    ThrottleBrake::APPSs_disagreement_implausibility_timer.Enable();
  }
};

/**
 * @brief Returns true if brake is pressed, false otherwise
 *
 * @return bool
 */
bool ThrottleBrake::is_brake_pressed() {
  // true: pressed
  // false: not pressed
  return ThrottleBrake::brake_pressed;
};

/**
 * @brief Sets true if both brake is pressed and throttle is >25%,
 *        Sets false after throttle returns to <=5% (regardless of brake) or otherwise
 *        (EV.4.7)
 *
 * @return
 */
void ThrottleBrake::check_BPPC_implausibility() {
  float APPS1_percentage =
      ((ThrottleBrake::APPS1_adc - static_cast<int32_t>(Bounds::APPS1_ADC_MIN)) * 100.0) /
      static_cast<int32_t>(Bounds::APPS1_ADC_SPAN);
  float APPS2_percentage =
      ((static_cast<int32_t>(Bounds::APPS2_ADC_MAX) - ThrottleBrake::APPS2_adc) * 100.0) /
      static_cast<int32_t>(Bounds::APPS2_ADC_SPAN);
  float percentage_diff = APPS1_percentage - APPS2_percentage;
  // Serial.print("percentage_diff: ");
  // Serial.println(percentage_diff);
  if (ThrottleBrake::is_brake_pressed() && ((percentage_diff > 25.0) || percentage_diff < -25.0)) {
    ThrottleBrake::BPPC_implausibility_present = true;
  }
  if (percentage_diff < 5.0 && percentage_diff > -5.0) {
    ThrottleBrake::BPPC_implausibility_present = false;
  }
};

/**
 * @brief Check for implausibility and set throttle/brake CAN signals accordingly
 *
 * @return void
 */
void ThrottleBrake::update_throttle_brake_CAN_signals() {
  ThrottleBrake::APPS1_Throttle = ThrottleBrake::APPS1_throttle_scaled;
  ThrottleBrake::APPS2_Throttle = ThrottleBrake::APPS2_throttle_scaled;
  ThrottleBrake::Front_Brake_Pressure = ThrottleBrake::front_brake_scaled;
  ThrottleBrake::Rear_Brake_Pressure = ThrottleBrake::rear_brake_scaled;
  ThrottleBrake::Brake_Pressed = ThrottleBrake::brake_pressed;
  ThrottleBrake::Implausibility_Present = ThrottleBrake::is_implausibility_present();
};

void ThrottleBrake::print_throttle_info() {
  // Serial.print(" APPS1 ADC: ");
  // Serial.print(ThrottleBrake::APPS1_ADC);
  // Serial.print(" APPS2 ADC: ");
  // Serial.print(ThrottleBrake::APPS2_ADC);

  // Serial.print(" APPS1: ");
  // Serial.print(ThrottleBrake::APPS1_throttle);
  // Serial.print(" APPS2: ");
  // Serial.print(ThrottleBrake::APPS2_throttle);
  Serial.print(" Front Brake: ");
  Serial.print(ThrottleBrake::front_brake_adc);
  // Serial.print("brake pressed: ");
  // Serial.print(ThrottleBrake::brake_pressed);
  // Serial.print(" Rear Brake: ");
  // Serial.print(ThrottleBrake::rear_brake);
  // Serial.print(" APPS2 ADC: ");
  // Serial.print(ThrottleBrake::APPS2_ADC, BIN);
  // Serial.print(" APPS2 ADC: ");
  // Serial.print(ThrottleBrake::APPS2_ADC);
  // Serial.print("APPS2 ADC: ");
  // Serial.println(ThrottleBrake::APPS2_ADC);
  // Serial.print("APPS1 Scaled (CAN): ");
  // Serial.println(ThrottleBrake::APPS1_throttle);
  // Serial.print("APPS2 Scaled (CAN): ");
  // Serial.println(ThrottleBrake::APPS2_throttle);
}
