#include <unity.h>
#include "throttle_brake_driver.hpp"
#ifdef ESP32
#include "esp_can.h"
#endif

// instantiate CAN bus
ESPCAN drive_bus{};


void setUp(void) {

}

void tearDown(void) {

}

void testIsBreakPressed() {
    ThrottleBrake throttle_brake(drive_bus, 1001);
    TEST_ASSERT_EQUAL(throttle_brake.is_brake_pressed(), true);

    ThrottleBrake throttle_brake(drive_bus, 999);
    TEST_ASSERT_EQUAL(throttle_brake.is_brake_pressed(), false);

}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(testIsBreakPressed);

    return UNITY_END();

}