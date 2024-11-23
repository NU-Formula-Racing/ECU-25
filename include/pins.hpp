#pragma once

// to get pin/bound number as an int:
// constexpr int pin = static_cast<int>(Pins::PIN_NAME);

// use GPIO names
enum class Pins {
    APPS1_CS_PIN = 0,
    APPS2_CS_PIN = 1,
    FRONT_BRAKE_CS_PIN = 2,
    REAR_BRAKE_CS_PIN = 3,
    BRAKE_VALID_PIN = 4,
    DRIVE_LEVER_PIN = 5,
    TS_ACTIVE_PIN = 6
};
