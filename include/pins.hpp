#pragma once

// to get pin/bound number as an int:
// constexpr int pin = static_cast<int>(Pins::PIN_NAME);

// use GPIO names
enum class Pins : uint8_t {
    APPS1_CS_PIN = 19,
    APPS2_CS_PIN = 21,
    FRONT_BRAKE_CS_PIN = 22,
    REAR_BRAKE_CS_PIN = 23,
    BRAKE_VALID_PIN = 32,
    READY_TO_DRIVE_SWITCH = 26,
    TS_ACTIVE_PIN = 27,
    SPI_CLK = 25,
    SPI_MISO = 18,
    SPI_MOSI = 34
};
