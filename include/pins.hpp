#pragma once

// ** to get bound/pin number as an int: **
// constexpr int pin = static_cast<int>(Pins::PIN_NAME);

// change specific bounds after testing with sensors in pedalbox
enum class Bounds : uint16_t {
    APPS1_MIN = 1456,
    APPS1_MAX = 4095,
    APPS1_RANGE = APPS1_MAX - APPS1_MIN,
    APPS2_MIN = 1456,
    APPS2_MAX = 4095,
    APPS2_RANGE = APPS2_MAX - APPS2_MIN,
    APPS1_RANGE = 2639,
    APPS2_MIN = 1456,
    APPS2_MAX = 4095,
    APPS2_RANGE = 2639,
    FRONT_BRAKE_MIN = 0,
    FRONT_BRAKE_MAX = 32767,
    FRONT_BRAKE_RANGE = FRONT_BRAKE_MAX - FRONT_BRAKE_MIN,
    REAR_BRAKE_MIN = 0,
    REAR_BRAKE_MAX = 32767,
    REAR_BRAKE_RANGE = REAR_BRAKE_MAX - REAR_BRAKE_MIN,
    BRAKE_PRESSED_THRESHOLD = 1000, // actual threshold is TBD, need to test with brake sensors
    BRAKE_DEFAULT_HIGH_INPUT_RAW_ADC_THRESHOLD = 4000,
    BRAKE_DEFAULT_LOW_INPUT_RAW_ADC_THRESHOLD = 1500
};

// use GPIO names
enum class Pins : uint8_t {
    APPS1_CS_PIN = 0,
    APPS2_CS_PIN = 0,
    FRONT_BRAKE_CS_PIN = 0,
    REAR_BRAKE_CS_PIN = 0,
    BRAKE_VALID_PIN = 0,
    DRIVE_LEVER_PIN = 0,
    TS_ACTIVE_PIN = 0
};
