#include <unity.h>

#include <cmath>

#include "LUT.cpp"

void setUp(void) {}

void tearDown(void) {}

void test_testing_framework(void) { TEST_ASSERT_EQUAL(true, true); }

void test_zero_all(void) {
  int32_t torque = LUT::calculate_accel_torque(0, 0, 0, 0);
  TEST_ASSERT(torque == 0);
}

void test_zero_throttle_others_random() {
  TEST_ASSERT(LUT::calculate_accel_torque(56, 45, 67, 0) == 0);
}

void test_throttle_with_reasonable_temps() {
  TEST_ASSERT_EQUAL(LUT::calculate_accel_torque(98, 33, 3, 2000),
                    (int32_t)(0.9953921568627451 * 235000));
}

void test_with_interpolation_for_all_LUTs() {
  int32_t torque = (int32_t)roundf(0.65 * 0.2 * 0.975 * 0.5381372549019607 * 235000.0);
  TEST_ASSERT_EQUAL(LUT::calculate_accel_torque(122, 56, 75, 860), torque);
}

/* ---------------- Extra unit tests (no cooling helpers) ------------------ */
/*  Paste these above runUnityTests() and add the RUN_TEST lines shown.      */

// ---------- calculate_accel_torque() edge behaviour ----------

// 1. Keys *below* all LUT ranges should clamp to the first entry
void test_calc_accel_below_range_clamps(void) {
  // Temps far below range, throttle far below range → expect zero torque
  TEST_ASSERT_EQUAL_INT32(0, LUT::calculate_accel_torque(-50, -50, -50, -10));
}

// 2. Keys *above* all LUT ranges should clamp to the last entry
void test_calc_accel_above_range_clamps(void) {
  // Temps at 250 °C clamp to 0 modifier; throttle > max clamps to 1.0
  TEST_ASSERT_EQUAL_INT32(0, LUT::calculate_accel_torque(250, 250, 250, 6000));
}

// 3. scale must round‑to‑nearest (up)
void test_scale_rounds_up(void) {
  float m = 0.33334f;        // 26 667.2 mA raw
  int32_t expected = 26667;  // roundf → 26667
  TEST_ASSERT_EQUAL_INT32(expected, LUT::scale(m, 80000));
}

// 4. scale must round‑to‑nearest (down / ties to even IEEE‑style)
void test_scale_rounds_down(void) {
  float m = 0.333332f;       // 26 666.56 mA raw
  int32_t expected = 26667;  // still rounds to 26667 with roundf
  TEST_ASSERT_EQUAL_INT32(expected, LUT::scale(m, 80000));
}

// 5. Interpolation sanity for all four LUTs mid‑range
void test_calc_accel_midpoints(void) {
  /* 85 °C IGBT  → 0.95
     48 °C Batt  → 0.875
     78 °C Motor → 0.975
     Throttle 900 → 0.515 (linearly between 819→0.51 and 921→0.58)
  */
  float ref_mod = 1 * 0.85 * 0.96 * 0.5655882352941176;
  int32_t ref = LUT::scale(ref_mod, 235000);
  TEST_ASSERT_EQUAL_INT32(ref, LUT::calculate_accel_torque(85, 48, 78, 900));
}

// ---------- lookup() exact‑hit and interpolation ----------

// 6. Exact‑hit on IGBT LUT returns precisely stored value
void test_lookup_exact_hit(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.75f, LUT::lookup(120, LUT::IGBTTemp2Modifier_LUT));
}

// 7. Interpolated value between two IGBT entries (115 °C between 110→0.9 & 120→0.75)
void test_lookup_interpolated_igbt(void) {
  float v = LUT::lookup(115, LUT::IGBTTemp2Modifier_LUT);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.825f, v);
}

// 8. Interpolated value for throttle LUT (halfway between 102 & 105)
void test_lookup_interpolated_throttle(void) {
  float v = LUT::lookup(103, LUT::AccelThrottle2Modifier_LUT);  // 0.03 → 0.09
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.030582524271844658, v);
}


// -- Out of bound test for Pump Duty cycle --

// 9. Motor below minimum LUT key (-50°C)
void test_below_min_Pump_Duty_Cycle_motor(void) {
  float v_low_motor = LUT::lookup(-50, LUT::MotorTemp2PumpDutyCycle_LUT);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, v_low_motor);
}

void test_below_min_Pump_Duty_Cycle_IGBT(void) {
  float v_low_igbt = LUT::lookup(-50, LUT::IGBTTemp2PumpDutyCycle_LUT);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, v_low_igbt);
}
void test_below_min_Pump_Duty_Cycle_battery(void) {
  float v_low_battery = LUT::lookup(-50, LUT::BatteryTemp2PumpDutyCycle_LUT);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, v_low_battery);
}

// 10. Above maximum LUT key (200°C)
void test_above_max_Pump_Duty_Cycle_motor(void) {
  float v_high_motor = LUT::lookup(200, LUT::MotorTemp2PumpDutyCycle_LUT);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0f, v_high_motor);
}

void test_above_max_Pump_Duty_Cycle_IGBT(void) {
  float v_high_igbt = LUT::lookup(200, LUT::IGBTTemp2PumpDutyCycle_LUT);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0f, v_high_igbt);
}
void test_above_max_Pump_Duty_Cycle_battery(void) {
  float v_high_battery = LUT::lookup(200, LUT::BatteryTemp2PumpDutyCycle_LUT);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0f, v_high_battery);
}

void test_above_max_calculate_pump_duty_cycle(void) {
  uint8_t scaled_PDC = LUT::calculate_pump_duty_cycle(200, 200, 200);
  TEST_ASSERT_EQUAL_UINT8(255, scaled_PDC);
}

// Expect minimum duty cycle when all inputs are far below LUT range
void test_pump_duty_cycle_all_below_min_should_be_zero(void) {
  uint8_t scaled_PDC = LUT::calculate_pump_duty_cycle(-50, -50, -50);
  TEST_ASSERT_EQUAL_UINT8(0, scaled_PDC);
}

// Expect minimum duty cycle when all temperatures map to 0 in the LUT
void test_pump_duty_cycle_all_map_to_zero(void) {
  uint8_t scaled_PDC = LUT::calculate_pump_duty_cycle(10, 20, 20);
  TEST_ASSERT_EQUAL_UINT8(0, scaled_PDC);
}

// Expect maximum duty cycle when all inputs are above max LUT range
void test_pump_duty_cycle_all_above_max_should_be_255(void) {
  uint8_t scaled_PDC = LUT::calculate_pump_duty_cycle(200, 200, 200);
  TEST_ASSERT_EQUAL_UINT8(255, scaled_PDC);
}

// Expect max duty cycle when battery temp alone maps to 1.0
void test_pump_duty_cycle_battery_only_at_max(void) {
  uint8_t scaled_PDC = LUT::calculate_pump_duty_cycle(30, 50, 60);
  TEST_ASSERT_EQUAL_UINT8(255, scaled_PDC);
}

// Expect max duty cycle when IGBT and battery temps map to 1.0
void test_pump_duty_cycle_igbt_and_battery_at_max(void) {
  uint8_t scaled_PDC = LUT::calculate_pump_duty_cycle(30, 100, 60);
  TEST_ASSERT_EQUAL_UINT8(255, scaled_PDC);
}

// Expect max duty cycle when all components map to 1.0
void test_pump_duty_cycle_all_at_max(void) {
  uint8_t scaled_PDC = LUT::calculate_pump_duty_cycle(55, 100, 60);
  TEST_ASSERT_EQUAL_UINT8(255, scaled_PDC);
}

// Expect minimum fan duty cycle for temperature below LUT minimum
void test_fan_duty_cycle_below_min_should_be_zero(void) {
  uint8_t fan_dc = LUT::calculate_fan_duty_cycle(-20.0f);
  TEST_ASSERT_EQUAL_UINT8(0, fan_dc);
}

// Expect minimum fan duty cycle for all temperatures mapping to 0
void test_fan_duty_cycle_at_zero_region(void) {
  uint8_t fan_dc = LUT::calculate_fan_duty_cycle(10.0f);  // 10°C → 0.0f
  TEST_ASSERT_EQUAL_UINT8(0, fan_dc);
}

// Expect a low fan duty cycle at 25°C (0.05 * 255 ≈ 12.75)
void test_fan_duty_cycle_at_25C(void) {
  uint8_t fan_dc = LUT::calculate_fan_duty_cycle(25.0f);
  TEST_ASSERT_EQUAL_UINT8(13, fan_dc);  // Rounded
}

// Interpolated fan duty cycle between 30°C (0.15) and 35°C (0.4)
void test_fan_duty_cycle_interpolated_32C(void) {
  uint8_t fan_dc = LUT::calculate_fan_duty_cycle(32.0f);
  // Interpolated value ≈ 0.15 + (0.4 - 0.15) * ((32-30)/(35-30)) = 0.25
  // → 0.25 * 255 ≈ 63.75
  TEST_ASSERT_EQUAL_UINT8(64, fan_dc);
}

// Fan duty cycle should hit maximum at 60°C
void test_fan_duty_cycle_at_max_should_be_255(void) {
  uint8_t fan_dc = LUT::calculate_fan_duty_cycle(60.0f);
  TEST_ASSERT_EQUAL_UINT8(255, fan_dc);
}

// Above max: 100°C → clamp to 60°C
void test_fan_duty_cycle_above_max_should_be_255(void) {
  uint8_t fan_dc = LUT::calculate_fan_duty_cycle(100.0f);
  TEST_ASSERT_EQUAL_UINT8(255, fan_dc);
}

int runUnityTests(void) {
  UNITY_BEGIN();
  RUN_TEST(test_testing_framework);
  RUN_TEST(test_zero_all);
  RUN_TEST(test_zero_throttle_others_random);
  RUN_TEST(test_throttle_with_reasonable_temps);
  RUN_TEST(test_with_interpolation_for_all_LUTs);
  RUN_TEST(test_calc_accel_below_range_clamps);
  RUN_TEST(test_calc_accel_above_range_clamps);
  RUN_TEST(test_scale_rounds_up);
  RUN_TEST(test_scale_rounds_down);
  RUN_TEST(test_calc_accel_midpoints);
  RUN_TEST(test_lookup_exact_hit);
  RUN_TEST(test_lookup_interpolated_igbt);
  RUN_TEST(test_lookup_interpolated_throttle);
  RUN_TEST(test_below_min_Pump_Duty_Cycle_motor);
  RUN_TEST(test_below_min_Pump_Duty_Cycle_IGBT);
  RUN_TEST(test_below_min_Pump_Duty_Cycle_battery);
  RUN_TEST(test_above_max_Pump_Duty_Cycle_motor);
  RUN_TEST(test_above_max_Pump_Duty_Cycle_IGBT);
  RUN_TEST(test_above_max_Pump_Duty_Cycle_battery);
  RUN_TEST(test_above_max_calculate_pump_duty_cycle);
  RUN_TEST(test_pump_duty_cycle_all_below_min_should_be_zero);
  RUN_TEST(test_pump_duty_cycle_all_map_to_zero);
  RUN_TEST(test_pump_duty_cycle_all_above_max_should_be_255);
  RUN_TEST(test_pump_duty_cycle_battery_only_at_max);
  RUN_TEST(test_pump_duty_cycle_igbt_and_battery_at_max);
  RUN_TEST(test_pump_duty_cycle_all_at_max);
  RUN_TEST(test_fan_duty_cycle_below_min_should_be_zero);
  RUN_TEST(test_fan_duty_cycle_at_zero_region);
  RUN_TEST(test_fan_duty_cycle_at_25C);
  RUN_TEST(test_fan_duty_cycle_interpolated_32C);
  RUN_TEST(test_fan_duty_cycle_at_max_should_be_255);
  RUN_TEST(test_fan_duty_cycle_above_max_should_be_255);

  return UNITY_END();
}

int main() { return runUnityTests(); }