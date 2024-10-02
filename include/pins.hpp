// change specific bounds after testing with sensors in pedalbox
enum class Bounds {
    APPS1_MIN = 0,
    APPS1_MAX = 32767,
    APPS2_MIN = 0,
    APPS2_MAX = 32767,
    FRONT_BRAKE_MIN = 0,
    FRONT_BRAKE_MAX = 32767,
    REAR_BRAKE_MIN = 0,
    REAR_BRAKE_MAX = 32767
};

// use GPIO names
enum class Pins {
    APPS1_CS_PIN = 0,
    APPS2_CS_PIN = 0,
    FRONT_BRAKE_CS_PIN = 0,
    REAR_BRAKE_CS_PIN = 0,
    BRAKE_VALID_PIN = 0,
    DRIVE_LEVER_PIN = 0
};