#include <Arduino.h>
#include <SPI.h>

#include "throttle_brake_driver.hpp"
#include "pins.hpp"

ThrottleBrake::ThrottleBrake(ICAN &can_interface_, VirtualTimer &APPSs_disagreement_implausibility_timer_, VirtualTimer &brake_shorted_or_opened_implausibility_timer_) :
    CAN_interface(can_interface_),
    APPSs_disagreement_implausibility_timer(APPSs_disagreement_implausibility_timer_),
    brake_shorted_or_opened_implausibility_timer(brake_shorted_or_opened_implausibility_timer_) {};

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

    // Set implausibilities to false:
    ThrottleBrake::APPSs_disagreement_implausibility_present = false;
    ThrottleBrake::BPPC_implausibility_present = false;
    ThrottleBrake::brake_shorted_or_opened_implausibility_present = false;

    SPI.begin(static_cast<uint8_t>(Pins::SPI_CLK), static_cast<uint8_t>(Pins::SPI_MISO), static_cast<uint8_t>(Pins::SPI_MOSI));

    initialize_CS_pins();

    pinMode(static_cast<uint8_t>(Pins::BRAKE_VALID_PIN), INPUT);
    
};

/**
 * @brief Reads data from ADCs and stores RAW sensor data (in ADC counts) in class variables  
 */
void ThrottleBrake::read_from_SPI_ADCs() {
    
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
    // 1000000: clock rate
    // MSBFIRST: Most significant bit first

    digitalWrite(static_cast<uint8_t>(Pins::APPS1_CS_PIN), LOW);
    ThrottleBrake::APPS1_RAW = (SPI.transfer16(0x0000) << 2) >> 4;
    digitalWrite(static_cast<uint8_t>(Pins::APPS1_CS_PIN), HIGH);

    digitalWrite(static_cast<uint8_t>(Pins::APPS2_CS_PIN), LOW);
    ThrottleBrake::APPS2_RAW = (SPI.transfer16(0x0000) << 2) >> 4;
    digitalWrite(static_cast<uint8_t>(Pins::APPS2_CS_PIN), HIGH);

    digitalWrite(static_cast<uint8_t>(Pins::FRONT_BRAKE_CS_PIN), LOW);
    ThrottleBrake::front_break_RAW = (SPI.transfer16(0x0000) << 2) >> 4;
    digitalWrite(static_cast<uint8_t>(Pins::FRONT_BRAKE_CS_PIN), HIGH);

    digitalWrite(static_cast<uint8_t>(Pins::REAR_BRAKE_CS_PIN), LOW);
    ThrottleBrake::rear_break_RAW = (SPI.transfer16(0x0000) << 2) >> 4;
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

    int16_t safe_APPS1_RAW = get_safe_RAW(ThrottleBrake::APPS1_RAW, static_cast<int32_t>(Bounds::APPS1_RAW_MIN), static_cast<int32_t>(Bounds::APPS1_RAW_MAX));
    ThrottleBrake::APPS1_throttle = (safe_APPS1_RAW - static_cast<int32_t>(Bounds::APPS1_RAW_MIN)) / static_cast<int32_t>(Bounds::APPS1_RAW_SPAN) * 32767;

    int16_t safe_APPS2_RAW = get_safe_RAW(ThrottleBrake::APPS2_RAW, static_cast<int32_t>(Bounds::APPS2_RAW_MIN), static_cast<int32_t>(Bounds::APPS2_RAW_MAX));
    ThrottleBrake::APPS2_throttle = (static_cast<int32_t>(Bounds::APPS2_RAW_MAX) - safe_APPS2_RAW) / static_cast<int32_t>(Bounds::APPS2_RAW_SPAN) * 32767;

    int16_t safe_front_break_RAW = get_safe_RAW(ThrottleBrake::front_break_RAW, static_cast<int32_t>(Bounds::FRONT_BRAKE_RAW_MIN), static_cast<int32_t>(Bounds::FRONT_BRAKE_RAW_MAX));
    ThrottleBrake::front_brake = (safe_front_break_RAW - static_cast<int32_t>(Bounds::FRONT_BRAKE_RAW_MIN)) / static_cast<int32_t>(Bounds::FRONT_BRAKE_RAW_SPAN) * 32767;

    int16_t safe_rear_break_RAW = get_safe_RAW(ThrottleBrake::rear_break_RAW, static_cast<int32_t>(Bounds::REAR_BRAKE_RAW_MIN), static_cast<int32_t>(Bounds::REAR_BRAKE_RAW_MAX));
    ThrottleBrake::rear_brake = (safe_rear_break_RAW - static_cast<int32_t>(Bounds::REAR_BRAKE_RAW_MIN)) / static_cast<int32_t>(Bounds::REAR_BRAKE_RAW_SPAN) * 32767;

}

/**
 * @brief Returns APPS1 throttle value, scaled 0-32767
 *
 * @return int32_t
 */   
int16_t ThrottleBrake::get_throttle() {
    return ThrottleBrake::APPS1_throttle;
};

/**
 * @brief Returns true if any implausibility is present, false otherwise
 *
 * @return bool
 */
bool ThrottleBrake::is_implausibility_present() {
    return ThrottleBrake::APPSs_disagreement_implausibility_present || ThrottleBrake::brake_shorted_or_opened_implausibility_present || ThrottleBrake::BPPC_implausibility_present;
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
    if (digitalRead(static_cast<uint8_t>(Pins::BRAKE_VALID_PIN)) == static_cast<bool>(BrakeStatus::INVALID) && ThrottleBrake::brake_shorted_or_opened_implausibility_timer.GetTimerState() == VirtualTimer::State::kNotStarted) {
        ThrottleBrake::brake_shorted_or_opened_implausibility_timer.Start(millis());
    }
    else if (digitalRead(static_cast<uint8_t>(Pins::BRAKE_VALID_PIN)) == static_cast<bool>(BrakeStatus::VALID) && ThrottleBrake::brake_shorted_or_opened_implausibility_timer.GetTimerState() == VirtualTimer::State::kRunning) {
        ThrottleBrake::brake_shorted_or_opened_implausibility_timer.Disable();
        ThrottleBrake::brake_shorted_or_opened_implausibility_timer.Enable();
    }
    // if implausibility detected and timer already running, then do nothing (timer ticks in main)
    // if there is no implausibility detected and no current implausibility timer running, then do nothing
};

/**
 * @brief Returns true if APPS1 and APPS2 disagree by >10% for >100ms, 
 *        Returns False otherwise
 *        (T.4.2.4 - T.4.2.5)
 *
 * @return void
 */
void ThrottleBrake::check_APPSs_disagreement_implausibility() {
    // Convert APPS raw values to throttle percentages (0-100)
    float APPS1_percentage = ((ThrottleBrake::APPS1_RAW - static_cast<int32_t>(Bounds::APPS1_RAW_MIN)) * 100.0) / static_cast<int32_t>(Bounds::APPS1_RAW_SPAN);
    float APPS2_percentage = ((static_cast<int32_t>(Bounds::APPS2_RAW_MAX) - ThrottleBrake::APPS2_RAW) * 100.0) / static_cast<int32_t>(Bounds::APPS2_RAW_SPAN);
    int32_t APPS_diff = APPS1_percentage - APPS2_percentage; // Get percentage point difference between APPS values
    if ((APPS_diff > 10.0 || APPS_diff < -10.0) && ThrottleBrake::APPSs_disagreement_implausibility_timer.GetTimerState() == VirtualTimer::State::kNotStarted) {
        ThrottleBrake::APPSs_disagreement_implausibility_timer.Start(millis());
        
    } else if (!(APPS_diff > 10.0 || APPS_diff < -10.0) && ThrottleBrake::APPSs_disagreement_implausibility_timer.GetTimerState() == VirtualTimer::State::kRunning) {
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
    // check if the front brake value is over a certain threshold (actual threshold is TBD, need to test with brake sensors)
    // if yes: return true
    // if no: return false
    if (ThrottleBrake::front_break_RAW >= static_cast<int32_t>(Bounds::FRONT_BRAKE_RAW_PRESSED_THRESHOLD)) {
        ThrottleBrake::brake_pressed = true;
        return true;
    }
    ThrottleBrake::brake_pressed = false;
    return false;
};

/**
 * @brief Sets true if both brake is pressed and throttle is >25%, 
 *        Sets false after throttle returns to <=5% (regardless of brake) or otherwise 
 *        (EV.4.7)
 *
 * @return 
 */
void ThrottleBrake::check_BPPC_implausibility() {

    float APPS1_percentage = ((ThrottleBrake::APPS1_RAW - static_cast<int32_t>(Bounds::APPS1_RAW_MIN)) * 100.0) / static_cast<int32_t>(Bounds::APPS1_RAW_SPAN);
    if (ThrottleBrake::is_brake_pressed() && (APPS1_percentage > 25.0)) {
        ThrottleBrake::BPPC_implausibility_present = true;
    }
    if (APPS1_percentage < 5.0) {
        ThrottleBrake::BPPC_implausibility_present = false;
    }

};

/**
 * @brief Check for implausibility and set throttle/brake CAN signals accordingly
 *
 * @return void
 */
void ThrottleBrake::update_throttle_brake_CAN_signals() {
    ThrottleBrake::APPS1_Throttle = ThrottleBrake::APPS1_throttle;
    ThrottleBrake::APPS2_Throttle = ThrottleBrake::APPS2_throttle;
    ThrottleBrake::Front_Brake_Pressure = ThrottleBrake::front_brake;
    ThrottleBrake::Rear_Brake_Pressure = ThrottleBrake::rear_brake;
    ThrottleBrake::Brake_Pressed = ThrottleBrake::brake_pressed;
    ThrottleBrake::Implausibility_Present = ThrottleBrake::is_implausibility_present();
};
