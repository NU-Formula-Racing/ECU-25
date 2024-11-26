#include <unity.h>
#include "fsm.hpp"


void setUp(void) {

}

void tearDown(void) {

}

void test_change_state(void) {
    current_state = OFF;
    tsactive_switch = TSActive::Active;
    BMS_State = BMSState::kActive;
    change_state();
    TEST_ASSERT_EQUAL(N, current_state);
    ready_to_drive = Ready_To_Drive_State::Drive;
    change_state();
    TEST_ASSERT_EQUAL(DRIVE, current_state);
    ready_to_drive = Ready_To_Drive_State::Neutral;
    change_state();
    TEST_ASSERT_EQUAL(N, current_state);
    tsactive_switch = TSActive::Inactive;
    change_state();
    TEST_ASSERT_EQUAL(OFF, current_state);
    current_state = N;
    BMS_State = BMSState::kFault;
    change_state();
    TEST_ASSERT_EQUAL(OFF, current_state);
}

int runUnityTests(void) {
    UNITY_BEGIN();
    RUN_TEST(test_change_state);
    return UNITY_END();
}

void setup() {
  // put your setup code here, to run once:
    // init();
    delay(2000);
    runUnityTests();
}

void loop() {
  // put your main code here, to run repeatedly:

}