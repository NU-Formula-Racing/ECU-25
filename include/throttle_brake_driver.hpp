#pragma once

#include "can_interface.h"
#include "esp_can.h"
#include "virtualTimer.h"

// change specific bounds after testing with sensors in pedalbox:
enum class Bounds {

  APPS1_ADC_MIN = 240,
  APPS1_ADC_MAX = 980,
  APPS1_ADC_SPAN = APPS1_ADC_MAX - APPS1_ADC_MIN,

  APPS2_ADC_MIN = 1140,
  APPS2_ADC_MAX = 1865,

  APPS2_ADC_SPAN = APPS2_ADC_MAX - APPS2_ADC_MIN,

  FRONT_BRAKE_ADC_MIN = 1456,
  FRONT_BRAKE_ADC_MAX = 4095,
  FRONT_BRAKE_ADC_SPAN = FRONT_BRAKE_ADC_MAX - FRONT_BRAKE_ADC_MIN,

  REAR_BRAKE_ADC_MIN = 1456,
  REAR_BRAKE_ADC_MAX = 4095,
  REAR_BRAKE_ADC_SPAN = REAR_BRAKE_ADC_MAX - REAR_BRAKE_ADC_MIN,

  FRONT_BRAKE_ADC_PRESSED_THRESHOLD = 400,

  SHORTED_THRESHOLD = 50,
  OPEN_THRESHOLD = 2035,
  SENSOR_SCALED_MAX = 2047,

  // FIVE_PERCENT_THROTTLE = 1639,
  // TEN_PERCENT_THROTTLE = 3276,
  // TWENTY_FIVE_PERCENT_THROTTLE = 8191

};

enum class BrakeStatus { VALID = 1, INVALID = 0 };

// transfer function slope of sensor
enum class SensorSlope { POSITIVE = 1, NEGATIVE = 0 };

class ThrottleBrake {
  // functions we want accessible outside of this class (ie. called in main):
  // initialize(), get_APPS1(), is_brake_pressed(), is_implausibility_present(),
  // send_throttle_brake_CAN() functions we should make private (ie. not called anywhere but in the
  // throttle/brake class): read_ADCs(), get_APPS2(), get_front_brake(), get_rear_brake(),
  // is_brake_implausible(), is_10_percent_rule_implausible(), is_BPPC_implausible()
 public:
  ThrottleBrake(ICAN& can_interface_, VirtualTimerGroup& timer_group,
                VirtualTimer& APPSs_disagreement_implausibility_timer_,
                VirtualTimer& brake_shorted_or_opened_implausibility_timer_,
                VirtualTimer& APPSs_invalid_implausibility_timer_)
      : can_interface(can_interface_),
        timers(timer_group),
        APPSs_disagreement_implausibility_timer(APPSs_disagreement_implausibility_timer_),
        brake_shorted_or_opened_implausibility_timer(brake_shorted_or_opened_implausibility_timer_),
        APPSs_invalid_implausibility_timer(APPSs_invalid_implausibility_timer_) {};

  // init timers in initialize()
  // we want 1 single-use timer per implausibility check
  // we'll have a callback function fires when the timer reaches its limit (ie. 100ms)
  // this callback will set a corresponding implausibility flag in a private struct containting all
  // the implausibilities
  void initialize();                // initialize CS pins, SPI, and implausibility states
  void update_sensor_values();      // read from SPI ADCs and update throttle/brake values
  int16_t get_throttle() const;     // return scaled throttle value
  int16_t get_front_brake() const;  // return scaled front brake value
  void set_is_APPSs_disagreement_implausibility_present_to_true();       // callback
  void set_is_brake_shorted_or_opened_implausibility_present_to_true();  // callback
  void set_APPSs_invalid_implausibility_present_to_true();               // callback
  void check_for_implausibilities();
  bool is_implausibility_present() const;
  bool is_brake_pressed() const;
  void update_throttle_brake_CAN_signals();
  void print_throttle_info();

 private:
  ICAN& can_interface;
  VirtualTimerGroup& timers;

  VirtualTimer& APPSs_disagreement_implausibility_timer;
  VirtualTimer& brake_shorted_or_opened_implausibility_timer;
  VirtualTimer& APPSs_invalid_implausibility_timer;

  int16_t APPS1_adc;        // 12-bit ADC: 0-4095 chnge to _ADC
  int16_t APPS2_adc;        // 12-bit ADC: 0-4095
  int16_t front_brake_adc;  // 12-bit ADC: 0-4095
  int16_t rear_brake_adc;   // 12-bit ADC: 0-4095

  int16_t
      APPS1_throttle_scaled;  // throttle calculated from APPS1 and scaled 0-32767 change to _scaled
  int16_t APPS2_throttle_scaled;  // throttle calculated from APPS2 and scaled 0-32767
  int16_t front_brake_scaled;     // front brake scaled 0-32767
  int16_t rear_brake_scaled;      // rear brake scaled 0-32767

  bool APPSs_disagreement_implausibility_present;
  bool BPPC_implausibility_present;
  bool brake_shorted_or_opened_implausibility_present;
  bool APPSs_invalid_implausibility_present;

  void read_from_SPI_ADCs();

  bool brake_pressed;

  void initialize_CS_pin(uint8_t CS_pin);
  void initialize_CS_pins();

  int16_t get_safe_RAW(int16_t RAW, int16_t projected_min, int16_t projected_max);

  void check_BPPC_implausibility();
  void check_brake_shorted_or_opened_implausibility();
  void check_APPSs_valid_implausibility();
  void check_APPSs_disagreement_implausibility();
  bool check_APPSs_validity() const;

  int16_t scale_ADC_input(int16_t ADC_input, int16_t ADC_min, int16_t ADC_max, int16_t ADC_span,
                          SensorSlope slope);

  const uint32_t kTransmissionIDThrottle = 0x202;        // CAN msg address, get this from DBC
  const uint32_t kTransmissionIDBrake = 0x203;           // CAN msg address, get this from DBC
  const uint32_t kTransmissionIDImplausibility = 0x204;  // CAN msg address, get this from DBC
  // CAN signals & msgs
  // tx: throttle percent, front brake, rear brake, brake pressed, implausibility present
  CANSignal<int16_t, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true>
      APPS1_Throttle{};
  CANSignal<int16_t, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true>
      APPS2_Throttle{};
  CANSignal<int16_t, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true>
      Front_Brake_Pressure{};
  CANSignal<int16_t, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true>
      Rear_Brake_Pressure{};
  CANSignal<bool, 32, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      Brake_Pressed{};
  CANSignal<bool, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      CAN_Implausibility_Present{};
  CANSignal<bool, 8, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      CAN_APPSs_Disagreement_Imp{};
  CANSignal<bool, 16, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      CAN_BPPC_Imp{};
  CANSignal<bool, 24, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      CAN_Brake_invalid_Imp{};
  CANSignal<bool, 32, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false>
      CAN_APPSs_Invalid_Imp{};
  CANTXMessage<2> ECU_Throttle{can_interface,  kTransmissionIDThrottle, 4, 100, timers,
                               APPS1_Throttle, APPS2_Throttle};
  CANTXMessage<3> ECU_Brake{
      can_interface,       kTransmissionIDBrake, 5, 100, timers, Front_Brake_Pressure,
      Rear_Brake_Pressure, Brake_Pressed};
  CANTXMessage<5> ECU_Implausibility{can_interface,
                                     kTransmissionIDImplausibility,
                                     5,
                                     100,
                                     timers,
                                     CAN_Implausibility_Present,
                                     CAN_APPSs_Disagreement_Imp,
                                     CAN_BPPC_Imp,
                                     CAN_Brake_invalid_Imp,
                                     CAN_APPSs_Invalid_Imp};
};
