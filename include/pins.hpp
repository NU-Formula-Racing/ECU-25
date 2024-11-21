#pragma once

// to get bound/pin number as an int: constexpr int pin = static_cast<int>(Pins::PIN_NAME);

// change specific bounds after testing with sensors in pedalbox
enum class Bounds : uint16_t {
    APPS1_RAW_MIN = 1456,
    APPS1_RAW_MAX = 4095,
    APPS1_RAW_SPAN = APPS1_RAW_MAX - APPS1_RAW_MIN,
    APPS2_RAW_MIN = 1456,
    APPS2_RAW_MAX = 4095,
    APPS2_RAW_SPAN = APPS2_RAW_MAX - APPS2_RAW_MIN,
    FRONT_BRAKE_RAW_MIN = 1456,
    FRONT_BRAKE_RAW_MAX = 4095,
    FRONT_BRAKE_RAW_SPAN = FRONT_BRAKE_RAW_MAX - FRONT_BRAKE_RAW_MIN,
    REAR_BRAKE_RAW_MIN = 1456,
    REAR_BRAKE_RAW_MAX = 4095,
    REAR_BRAKE_RAW_SPAN = REAR_BRAKE_RAW_MAX - REAR_BRAKE_RAW_MIN,
    FRONT_BRAKE_RAW_PRESSED_THRESHOLD = 1000,
    FRONT_BRAKE_RAW_DEFAULT_LOW_THRESHOLD = 1500,
    FRONT_BRAKE_RAW_DEFAULT_HIGH_THRESHOLD = 4000
};

// use GPIO names
enum class Pins : uint8_t {
    APPS1_SS_PIN = 0,
    APPS2_SS_PIN = 1,
    FRONT_BRAKE_SS_PIN = 2,
    REAR_BRAKE_SS_PIN = 3,
    BRAKE_VALID_PIN = 4,
    DRIVE_LEVER_PIN = 5,
    TS_ACTIVE_PIN = 6
};
