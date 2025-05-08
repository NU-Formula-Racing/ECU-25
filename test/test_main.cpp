#include <unity.h>

#include <cmath>

#include "LUT.cpp"
// #include "test_LUT.cpp"

void setUp(void) {}

void tearDown(void) {}

void test_testing_framework(void) { TEST_ASSERT_EQUAL(true, true); }

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

void test_pump_duty_cycle_not_max_or_min(void) {
  uint8_t scaled_PDC = LUT::calculate_pump_duty_cycle(50, 60, 35);
  TEST_ASSERT_EQUAL_UINT8(77, scaled_PDC);
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

// Unit tests for LUT::get_throttle_difference
void test_throttle_diff_zero_input(void) {
  TEST_ASSERT_EQUAL_INT16(0, LUT::get_throttle_difference(0, 2047, 0));
}
void test_throttle_diff_exact_key(void) {
  TEST_ASSERT_EQUAL_INT16(-189, LUT::get_throttle_difference(200, 2047, 1000));
}
void test_throttle_diff_interpolation(void) {
  TEST_ASSERT_EQUAL_INT16(28, LUT::get_throttle_difference(100, 2047, 300));
}
void test_throttle_diff_below_min_rpm(void) {
  TEST_ASSERT_EQUAL_INT16(50, LUT::get_throttle_difference(50, 2047, -100));
}
void test_throttle_diff_above_max_rpm(void) {
  TEST_ASSERT_EQUAL_INT16(-362, LUT::get_throttle_difference(150, 2047, 20000));
}
void test_throttle_diff_zero_max(void) {
  TEST_ASSERT_EQUAL_INT16(-359, LUT::get_throttle_difference(30, 2047, 1000));
}
void test_throttle_diff_mostly_pressed_slow_real(void) {
  TEST_ASSERT_EQUAL_INT16(1586, LUT::get_throttle_difference(1975, 2047, 1000));
}
void test_throttle_diff_large_values(void) {
  TEST_ASSERT_EQUAL_INT16(1558, LUT::get_throttle_difference(2047, 2047, 2000));
}
void test_throttle_diff_mixed_negative(void) {
  TEST_ASSERT_EQUAL_INT16(728, LUT::get_throttle_difference(1234, 2047, 2400));
}
void test_throttle_diff_negative_max_input(void) {
  TEST_ASSERT_EQUAL_INT16(-512, LUT::get_throttle_difference(0, 2047, 10000));
}

// Unit tests for LUT::get_torque_mods
void test_torque_mods_zero_input(void) {
  auto mods = LUT::get_torque_mods(0, 2047, 0, false);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, mods.first);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, mods.second);
}
void test_torque_mods_exact_key(void) {
  auto mods = LUT::get_torque_mods(102, 2047, 0, false);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.03f, mods.first);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, mods.second);
}
void test_torque_mods_interpolation(void) {
  auto mods = LUT::get_torque_mods(1539, 2047, 0, false);
  float exp = LUT::lookup(1539, LUT::AccelThrottle2Modifier_LUT);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, exp, mods.first);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0f, mods.second);
}
void test_torque_mods_below_min_diff(void) {
  auto mods = LUT::get_torque_mods(0, 2047, -100, false);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, mods.first);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, mods.second);
}
void test_torque_mods_above_max_diff(void) {
  auto mods = LUT::get_torque_mods(3000, 2047, 10000, false);
  float exp = LUT::lookup(2488, LUT::AccelThrottle2Modifier_LUT);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, exp, mods.first);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, mods.second);
}
void test_torque_mods_max_regen(void) {
  auto mods = LUT::get_torque_mods(0, 2047, 10000, false);
  float exp = LUT::lookup(512, LUT::RegenThrottle2Modifier_LUT);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, mods.first);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, exp, mods.second);
}
void test_torque_mods_accel_normal(void) {
  auto mods = LUT::get_torque_mods(500, 2047, 1000, false);
  float exp = LUT::lookup(111, LUT::AccelThrottle2Modifier_LUT);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, exp, mods.first);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0f, mods.second);
}
void test_torque_mods_fast_flooring(void) {
  auto mods = LUT::get_torque_mods(2047, 2047, 10000, false);
  float exp = LUT::lookup(1535, LUT::AccelThrottle2Modifier_LUT);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, exp, mods.first);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, mods.second);
}
void test_torque_mods_one_step(void) {
  auto mods = LUT::get_torque_mods(1, 2047, 0, false);
  float exp = LUT::lookup(1, LUT::AccelThrottle2Modifier_LUT);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, exp, mods.first);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0, mods.second);
}
void test_torque_mods_high_step(void) {
  auto mods = LUT::get_torque_mods(2046, 2047, 0, false);
  float exp = LUT::lookup(2046, LUT::AccelThrottle2Modifier_LUT);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, exp, mods.first);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0f, mods.second);
}
void test_torque_mods_stopped_brake_pressed(void) {
  auto mods = LUT::get_torque_mods(0, 2047, 0, true);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, mods.first);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, mods.second);
}

void test_torque_mods_regen_brake_pressed(void) {
  auto mods = LUT::get_torque_mods(0, 2047, 1600, true);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, mods.first);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, mods.second);
}

void test_torque_mods_accel_brake_pressed(void) {
  auto mods = LUT::get_torque_mods(1000, 2047, 1600, true);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, mods.first);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, mods.second);
}
// Unit tests for LUT::calculate_temp_mod
void test_temp_mod_nominal(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0f, LUT::calculate_temp_mod(0, 0, 0));
}
void test_temp_mod_extreme_degrade(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, LUT::calculate_temp_mod(200, 0, 0));
}
void test_temp_mod_interpolation_combined(void) {
  float product = 0.825f * 0.65f * 0.85f;
  TEST_ASSERT_FLOAT_WITHIN(1e-6, product, LUT::calculate_temp_mod(115, 53, 85));
}
void test_temp_mod_exact_keys(void) {
  float exp = 0.75f * 0.75f * 0.95f;
  TEST_ASSERT_FLOAT_WITHIN(1e-6, exp, LUT::calculate_temp_mod(120, 50, 80));
}
void test_temp_mod_negative_inputs(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0f, LUT::calculate_temp_mod(-10, -5, -1));
}
void test_temp_mod_all_zero_corner(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, LUT::calculate_temp_mod(150, 60, 120));
}
void test_temp_mod_mixed_combination(void) {
  float exp = 0.05f * 1.0f * 0.2f;
  TEST_ASSERT_FLOAT_WITHIN(1e-6, exp, LUT::calculate_temp_mod(140, 45, 100));
}
void test_temp_mod_half_interpolation(void) {
  float exp = 0.5f * 0.45f * 0.025f;
  TEST_ASSERT_FLOAT_WITHIN(1e-6, exp, LUT::calculate_temp_mod(125, 53, 115));
}
void test_temp_mod_boundary_low(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0f, LUT::calculate_temp_mod(0, 5, 0));
}
void test_temp_mod_boundary_high(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, LUT::calculate_temp_mod(150, 60, 120));
}

// Unit tests for LUT::calculate_torque_reqs
void test_calc_torque_reqs_full(void) {
  auto reqs = LUT::calculate_torque_reqs(1.0f, {1.0f, 1.0f});
  TEST_ASSERT_EQUAL_INT32(LUT::TorqueReqLimit::kAccelMax, reqs.first);
  TEST_ASSERT_EQUAL_INT32(LUT::TorqueReqLimit::kRegenMax, reqs.second);
}
void test_calc_torque_reqs_none(void) {
  auto reqs = LUT::calculate_torque_reqs(0.0f, {1.0f, 1.0f});
  TEST_ASSERT_EQUAL_INT32(0, reqs.first);
  TEST_ASSERT_EQUAL_INT32(0, reqs.second);
}
void test_calc_torque_reqs_half_temp(void) {
  int32_t halfAccel =
      static_cast<int32_t>(roundf(0.5f * static_cast<float>(LUT::TorqueReqLimit::kAccelMax)));
  int32_t halfRegen =
      static_cast<int32_t>(roundf(0.5f * static_cast<float>(LUT::TorqueReqLimit::kRegenMax)));
  auto reqs = LUT::calculate_torque_reqs(0.5f, {1.0f, 1.0f});
  TEST_ASSERT_EQUAL_INT32(halfAccel, reqs.first);
  TEST_ASSERT_EQUAL_INT32(halfRegen, reqs.second);
}
void test_calc_torque_reqs_half_mods(void) {
  int32_t halfAccel = static_cast<int32_t>(
      roundf(0.5f * 0.5f * static_cast<float>(LUT::TorqueReqLimit::kAccelMax)));
  int32_t halfRegen = static_cast<int32_t>(
      roundf(0.5f * 0.5f * static_cast<float>(LUT::TorqueReqLimit::kRegenMax)));
  auto reqs = LUT::calculate_torque_reqs(0.5f, {0.5f, 0.5f});
  TEST_ASSERT_EQUAL_INT32(halfAccel, reqs.first);
  TEST_ASSERT_EQUAL_INT32(halfRegen, reqs.second);
}
void test_calc_torque_reqs_asymmetric(void) {
  int32_t accel = static_cast<int32_t>(
      roundf(1.0f * 0.8f * static_cast<float>(LUT::TorqueReqLimit::kAccelMax)));
  int32_t regen = static_cast<int32_t>(
      roundf(1.0f * 0.6f * static_cast<float>(LUT::TorqueReqLimit::kRegenMax)));
  auto reqs = LUT::calculate_torque_reqs(1.0f, {0.8f, 0.6f});
  TEST_ASSERT_EQUAL_INT32(accel, reqs.first);
  TEST_ASSERT_EQUAL_INT32(regen, reqs.second);
}
void test_calc_torque_reqs_zero_temp(void) {
  auto reqs = LUT::calculate_torque_reqs(1.0f, {1.0f, 0.0f});
  TEST_ASSERT_EQUAL_INT32(LUT::TorqueReqLimit::kAccelMax, reqs.first);
  TEST_ASSERT_EQUAL_INT32(0, reqs.second);
}
void test_calc_torque_reqs_nonint(void) {
  float tm = 0.123f;
  float am = 0.456f;
  float rm = 0.789f;
  int32_t expA =
      static_cast<int32_t>(roundf(tm * am * static_cast<float>(LUT::TorqueReqLimit::kAccelMax)));
  int32_t expR =
      static_cast<int32_t>(roundf(tm * rm * static_cast<float>(LUT::TorqueReqLimit::kRegenMax)));
  auto reqs = LUT::calculate_torque_reqs(tm, {am, rm});
  TEST_ASSERT_EQUAL_INT32(expA, reqs.first);
  TEST_ASSERT_EQUAL_INT32(expR, reqs.second);
}
void test_calc_torque_reqs_zero_mods(void) {
  auto reqs = LUT::calculate_torque_reqs(1.0f, {0.0f, 0.0f});
  TEST_ASSERT_EQUAL_INT32(0, reqs.first);
  TEST_ASSERT_EQUAL_INT32(0, reqs.second);
}
void test_calc_torque_reqs_edge_values(void) {
  auto reqs = LUT::calculate_torque_reqs(0.001f, {0.001f, 0.001f});
  int32_t expA = static_cast<int32_t>(
      roundf(0.001f * 0.001f * static_cast<float>(LUT::TorqueReqLimit::kAccelMax)));
  int32_t expR = static_cast<int32_t>(
      roundf(0.001f * 0.001f * static_cast<float>(LUT::TorqueReqLimit::kRegenMax)));
  TEST_ASSERT_EQUAL_INT32(expA, reqs.first);
  TEST_ASSERT_EQUAL_INT32(expR, reqs.second);
}

// Integration tests combining full pipeline
void test_integration_nominal_full(void) {
  auto mods = LUT::get_torque_mods(2047, 2047, 10000);
  float tm = LUT::calculate_temp_mod(0, 0, 0);
  auto reqs = LUT::calculate_torque_reqs(tm, mods);
  int32_t expA =
      static_cast<int32_t>(roundf(mods.first * static_cast<float>(LUT::TorqueReqLimit::kAccelMax)));
  int32_t expR = static_cast<int32_t>(
      roundf(mods.second * static_cast<float>(LUT::TorqueReqLimit::kRegenMax)));
  TEST_ASSERT_EQUAL_INT32(expA, reqs.first);
  TEST_ASSERT_EQUAL_INT32(expR, reqs.second);
}
void test_integration_zero_throttle(void) {
  auto mods = LUT::get_torque_mods(0, 1000, 0);
  float tm = LUT::calculate_temp_mod(50, 25, 60);
  auto reqs = LUT::calculate_torque_reqs(tm, mods);
  TEST_ASSERT_EQUAL_INT32(0, reqs.first);
  TEST_ASSERT_EQUAL_INT32(0, reqs.second);
}
void test_integration_high_temp_zero_output(void) {
  auto mods = LUT::get_torque_mods(1000, 1000, 1000);
  float tm = LUT::calculate_temp_mod(150, 60, 120);
  auto reqs = LUT::calculate_torque_reqs(tm, mods);
  TEST_ASSERT_EQUAL_INT32(0, reqs.first);
  TEST_ASSERT_EQUAL_INT32(0, reqs.second);
}
void test_integration_negative_diff(void) {
  auto mods = LUT::get_torque_mods(0, 50, 1000);
  float tm = LUT::calculate_temp_mod(0, 0, 0);
  auto reqs = LUT::calculate_torque_reqs(tm, mods);
  TEST_ASSERT_EQUAL_INT32(0, reqs.first);
  TEST_ASSERT_EQUAL_INT32(0, reqs.second);
}
void test_integration_partial_pipeline(void) {
  auto mods = LUT::get_torque_mods(500, 500, 500);
  float tm = LUT::calculate_temp_mod(115, 53, 85);
  auto reqs = LUT::calculate_torque_reqs(tm, mods);
  int32_t expA = static_cast<int32_t>(
      roundf(tm * mods.first * static_cast<float>(LUT::TorqueReqLimit::kAccelMax)));
  int32_t expR = static_cast<int32_t>(
      roundf(tm * mods.second * static_cast<float>(LUT::TorqueReqLimit::kRegenMax)));
  TEST_ASSERT_EQUAL_INT32(expA, reqs.first);
  TEST_ASSERT_EQUAL_INT32(expR, reqs.second);
}
void test_integration_mixed_inputs(void) {
  auto mods = LUT::get_torque_mods(100, 200, 300);
  float tm = LUT::calculate_temp_mod(120, 50, 80);
  auto reqs = LUT::calculate_torque_reqs(tm, mods);
  int32_t expA = static_cast<int32_t>(
      roundf(tm * mods.first * static_cast<float>(LUT::TorqueReqLimit::kAccelMax)));
  int32_t expR = static_cast<int32_t>(
      roundf(tm * mods.second * static_cast<float>(LUT::TorqueReqLimit::kRegenMax)));
  TEST_ASSERT_EQUAL_INT32(expA, reqs.first);
  TEST_ASSERT_EQUAL_INT32(expR, reqs.second);
}
void test_integration_extreme_values(void) {
  auto mods = LUT::get_torque_mods(32767, 32767, 2000);
  float tm = LUT::calculate_temp_mod(140, 45, 100);
  auto reqs = LUT::calculate_torque_reqs(tm, mods);
  int32_t expA = static_cast<int32_t>(
      roundf(tm * mods.first * static_cast<float>(LUT::TorqueReqLimit::kAccelMax)));
  int32_t expR = static_cast<int32_t>(
      roundf(tm * mods.second * static_cast<float>(LUT::TorqueReqLimit::kRegenMax)));
  TEST_ASSERT_EQUAL_INT32(expA, reqs.first);
  TEST_ASSERT_EQUAL_INT32(expR, reqs.second);
}
void test_integration_small_fractional(void) {
  auto mods = LUT::get_torque_mods(10, 100, 20);
  float tm = LUT::calculate_temp_mod(115, 53, 85);
  auto reqs = LUT::calculate_torque_reqs(tm, mods);
  int32_t expA = static_cast<int32_t>(
      roundf(tm * mods.first * static_cast<float>(LUT::TorqueReqLimit::kAccelMax)));
  int32_t expR = static_cast<int32_t>(
      roundf(tm * mods.second * static_cast<float>(LUT::TorqueReqLimit::kRegenMax)));
  TEST_ASSERT_EQUAL_INT32(expA, reqs.first);
  TEST_ASSERT_EQUAL_INT32(expR, reqs.second);
}
void test_integration_boundary_cases(void) {
  auto mods = LUT::get_torque_mods(2046, 2047, 0);
  float tm = LUT::calculate_temp_mod(125, 53, 115);
  auto reqs = LUT::calculate_torque_reqs(tm, mods);
  int32_t expA = static_cast<int32_t>(
      roundf(tm * mods.first * static_cast<float>(LUT::TorqueReqLimit::kAccelMax)));
  int32_t expR = static_cast<int32_t>(
      roundf(tm * mods.second * static_cast<float>(LUT::TorqueReqLimit::kRegenMax)));
  TEST_ASSERT_EQUAL_INT32(expA, reqs.first);
  TEST_ASSERT_EQUAL_INT32(expR, reqs.second);
}

int runUnityTests(void) {
  UNITY_BEGIN();
  RUN_TEST(test_testing_framework);
  RUN_TEST(test_scale_rounds_up);
  RUN_TEST(test_scale_rounds_down);
  RUN_TEST(test_pump_duty_cycle_not_max_or_min);
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
  // throttle difference
  RUN_TEST(test_throttle_diff_zero_input);
  RUN_TEST(test_throttle_diff_exact_key);
  RUN_TEST(test_throttle_diff_interpolation);
  RUN_TEST(test_throttle_diff_below_min_rpm);
  RUN_TEST(test_throttle_diff_above_max_rpm);
  RUN_TEST(test_throttle_diff_zero_max);
  RUN_TEST(test_throttle_diff_mostly_pressed_slow_real);
  RUN_TEST(test_throttle_diff_large_values);
  RUN_TEST(test_throttle_diff_mixed_negative);
  RUN_TEST(test_throttle_diff_negative_max_input);
  // torque mods
  RUN_TEST(test_torque_mods_zero_input);
  RUN_TEST(test_torque_mods_exact_key);
  RUN_TEST(test_torque_mods_interpolation);
  RUN_TEST(test_torque_mods_below_min_diff);
  RUN_TEST(test_torque_mods_above_max_diff);
  RUN_TEST(test_torque_mods_max_regen);
  RUN_TEST(test_torque_mods_accel_normal);
  RUN_TEST(test_torque_mods_fast_flooring);
  RUN_TEST(test_torque_mods_stopped_brake_pressed);
  RUN_TEST(test_torque_mods_regen_brake_pressed);
  RUN_TEST(test_torque_mods_accel_brake_pressed);
  RUN_TEST(test_torque_mods_one_step);
  RUN_TEST(test_torque_mods_high_step);
  // temp mod
  RUN_TEST(test_temp_mod_nominal);
  RUN_TEST(test_temp_mod_extreme_degrade);
  RUN_TEST(test_temp_mod_interpolation_combined);
  RUN_TEST(test_temp_mod_exact_keys);
  RUN_TEST(test_temp_mod_negative_inputs);
  RUN_TEST(test_temp_mod_all_zero_corner);
  RUN_TEST(test_temp_mod_mixed_combination);
  RUN_TEST(test_temp_mod_half_interpolation);
  RUN_TEST(test_temp_mod_boundary_low);
  RUN_TEST(test_temp_mod_boundary_high);
  // torque reqs
  RUN_TEST(test_calc_torque_reqs_full);
  RUN_TEST(test_calc_torque_reqs_none);
  RUN_TEST(test_calc_torque_reqs_half_temp);
  RUN_TEST(test_calc_torque_reqs_half_mods);
  RUN_TEST(test_calc_torque_reqs_asymmetric);
  RUN_TEST(test_calc_torque_reqs_zero_temp);
  RUN_TEST(test_calc_torque_reqs_nonint);
  RUN_TEST(test_calc_torque_reqs_zero_mods);
  RUN_TEST(test_calc_torque_reqs_edge_values);
  // integration
  RUN_TEST(test_integration_nominal_full);
  RUN_TEST(test_integration_zero_throttle);
  RUN_TEST(test_integration_high_temp_zero_output);
  RUN_TEST(test_integration_negative_diff);
  RUN_TEST(test_integration_partial_pipeline);
  RUN_TEST(test_integration_mixed_inputs);
  RUN_TEST(test_integration_extreme_values);
  RUN_TEST(test_integration_small_fractional);
  RUN_TEST(test_integration_boundary_cases);

  return UNITY_END();
}

int main() { return runUnityTests(); }