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
<<<<<<< HEAD
    APPS1_CS_PIN = 19,
    APPS2_CS_PIN = 21,
    FRONT_BRAKE_CS_PIN = 22,
    REAR_BRAKE_CS_PIN = 23,
    BRAKE_VALID_PIN = 32,
    READY_TO_DRIVE_SWITCH = 26,
    TS_ACTIVE_PIN = 27
=======
    APPS1_CS_PIN = 0,
    APPS2_CS_PIN = 1,
    FRONT_BRAKE_CS_PIN = 2,
    REAR_BRAKE_CS_PIN = 3,
    BRAKE_VALID_PIN = 4,
    DRIVE_LEVER_PIN = 5,
    TS_ACTIVE_PIN = 6
>>>>>>> 0a2bb1c0526b72e97c21802f3f4ef6160d47f6da
};
