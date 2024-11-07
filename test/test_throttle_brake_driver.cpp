// test/native/test_throttle_brake_driver.cpp

#include <unity.h>
#include "throttle_brake_driver.hpp"
#include "pins.hpp"

// Mock for ICAN interface
class MockCAN : public ICAN {
public:
    // Implement virtual methods if any. For simplicity, assuming ICAN has no methods.
    // Add mock implementations as needed.
};

// Mock implementations of Arduino functions

void pinMode(uint8_t pin, uint8_t mode) {
    // Mock implementation: Do nothing or log if needed
}

unsigned long mock_millis = 0;

unsigned long millis() {
    return mock_millis;
}

// Function to manipulate mock_millis for testing time-dependent features
void set_mock_millis(unsigned long millis_val) {
    mock_millis = millis_val;
}

// Test variables
MockCAN mock_can;
ThrottleBrake* throttle_brake;

// setUp is called before each test
void setUp(void) {
    throttle_brake = new ThrottleBrake(mock_can);
    throttle_brake->initialize();
}

// tearDown is called after each test
void tearDown(void) {
    delete throttle_brake;
}

// Test initialization (pinMode calls)
void test_initialization(void) {
    // Since pinMode is mocked to do nothing, we can only ensure that initialize() runs without errors
    TEST_PASS();
}

// Test scaling of APPS1
void test_get_APPS1(void) {
    // Set APPS1_raw to a known value within range
    throttle_brake->APPS1_raw = static_cast<uint16_t>(Bounds::APPS1_MIN) + (static_cast<uint16_t>(Bounds::APPS1_RANGE) / 2);
    
    uint16_t scaled_APPS1 = throttle_brake->get_APPS1();
    uint16_t expected = (throttle_brake->APPS1_raw - static_cast<uint16_t>(Bounds::APPS1_MIN)) * 32767 / static_cast<uint16_t>(Bounds::APPS1_RANGE);
    
    TEST_ASSERT_EQUAL_UINT16(expected, scaled_APPS1);
}

// Test scaling of APPS2
void test_get_APPS2(void) {
    // Set APPS2_raw to a known value within range
    throttle_brake->APPS2_raw = static_cast<uint16_t>(Bounds::APPS2_MAX) - (static_cast<uint16_t>(Bounds::APPS2_RANGE) / 2);
    
    uint16_t scaled_APPS2 = throttle_brake::get_APPS2();
    uint16_t expected = (static_cast<uint16_t>(Bounds::APPS2_MAX) - throttle_brake->APPS2_raw) * 32767 / static_cast<uint16_t>(Bounds::APPS2_RANGE);
    
    TEST_ASSERT_EQUAL_UINT16(expected, scaled_APPS2);
}

// Test scaling of front brake
void test_get_front_brake(void) {
    // Set front_brake_raw to a known value within range
    throttle_brake->front_brake_raw = static_cast<uint16_t>(Bounds::FRONT_BRAKE_MIN) + (static_cast<uint16_t>(Bounds::FRONT_BRAKE_RANGE) / 2);
    
    uint16_t scaled_front_brake = throttle_brake->get_front_brake();
    uint16_t expected = (throttle_brake->front_brake_raw - static_cast<uint16_t>(Bounds::FRONT_BRAKE_MIN)) * 32767 / static_cast<uint16_t>(Bounds::FRONT_BRAKE_RANGE);
    
    TEST_ASSERT_EQUAL_UINT16(expected, scaled_front_brake);
}

// Test scaling of rear brake
void test_get_rear_brake(void) {
    // Set rear_brake_raw to a known value within range
    throttle_brake->rear_brake_raw = static_cast<uint16_t>(Bounds::REAR_BRAKE_MIN) + (static_cast<uint16_t>(Bounds::REAR_BRAKE_RANGE) / 2);
    
    uint16_t scaled_rear_brake = throttle_brake->get_rear_brake();
    uint16_t expected = (throttle_brake->rear_brake_raw - static_cast<uint16_t>(Bounds::REAR_BRAKE_MIN)) * 32767 / static_cast<uint16_t>(Bounds::REAR_BRAKE_RANGE);
    
    TEST_ASSERT_EQUAL_UINT16(expected, scaled_rear_brake);
}

// Test brake pressed status
void test_is_brake_pressed(void) {
    // Case 1: Brake not pressed
    throttle_brake->front_brake = static_cast<uint16_t>(Bounds::BRAKE_PRESSED_THRESHOLD) - 1;
    bool brake_pressed = throttle_brake->is_brake_pressed();
    TEST_ASSERT_FALSE(brake_pressed);
    TEST_ASSERT_FALSE(throttle_brake->brake_pressed_signal);
    
    // Case 2: Brake pressed
    throttle_brake->front_brake = static_cast<uint16_t>(Bounds::BRAKE_PRESSED_THRESHOLD) + 100;
    brake_pressed = throttle_brake->is_brake_pressed();
    TEST_ASSERT_TRUE(brake_pressed);
    TEST_ASSERT_TRUE(throttle_brake->brake_pressed_signal);
}

// Test implausibility presence
void test_is_implausibility_present(void) {
    // Initially, no implausibility
    TEST_ASSERT_FALSE(throttle_brake->is_implausibility_present());
    
    // Simulate brake implausible
    throttle_brake->front_brake_raw = static_cast<uint16_t>(Bounds::BRAKE_DEFAULT_HIGH_INPUT_RAW_ADC_THRESHOLD) + 1;
    TEST_ASSERT_TRUE(throttle_brake->is_implausibility_present());
    
    // Reset
    throttle_brake->front_brake_raw = static_cast<uint16_t>(Bounds::FRONT_BRAKE_MIN) + 100;
    TEST_ASSERT_FALSE(throttle_brake->is_implausibility_present());
}

// Test 10 percent rule implausibility
void test_is_10_percent_rule_implausible(void) {
    // Set APPS1_raw and APPS2_raw to have less than 10% difference
    throttle_brake->APPS1_raw = static_cast<uint16_t>(Bounds::APPS1_MIN) + (static_cast<uint16_t>(Bounds::APPS1_RANGE) / 2);
    throttle_brake->APPS2_raw = throttle_brake->APPS1_raw + (static_cast<uint16_t>(Bounds::APPS2_RANGE) / 20); // 5% difference
    
    TEST_ASSERT_FALSE(throttle_brake->is_10_percent_rule_implausible());
    
    // Set APPS1_raw and APPS2_raw to have more than 10% difference
    throttle_brake->APPS2_raw = throttle_brake->APPS1_raw + (static_cast<uint16_t>(Bounds::APPS2_RANGE) / 5); // 20% difference
    
    // Simulate time progression
    set_mock_millis(1000); // Start time
    TEST_ASSERT_FALSE(throttle_brake->is_10_percent_rule_implausible());
    
    set_mock_millis(1100); // 100ms later
    TEST_ASSERT_TRUE(throttle_brake->is_10_percent_rule_implausible());
}

// Test BPPC implausibility
void test_is_BPPC_implausible(void) {
    // Initially, no implausibility
    TEST_ASSERT_FALSE(throttle_brake->is_BPPC_implausible());
    
    // Set conditions to trigger BPPC implausibility
    throttle_brake->front_brake = static_cast<uint16_t>(Bounds::FRONT_BRAKE_MIN) + 1;
    throttle_brake->rear_brake = static_cast<uint16_t>(Bounds::REAR_BRAKE_MIN) + 1;
    throttle_brake->APPS1 = 9000; // > 8191
    
    TEST_ASSERT_TRUE(throttle_brake->is_BPPC_implausible());
    
    // Simulate throttle dropping to <=5%
    throttle_brake->APPS1 = 1500; // <= 1638
    TEST_ASSERT_FALSE(throttle_brake->is_BPPC_implausible());
}
