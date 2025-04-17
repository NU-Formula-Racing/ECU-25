#include "brake.hpp"
#include "throttle.hpp"
#include "virtualTimer.h"

struct ImplausibilitiesData {
  bool any_implausibility_present;
  bool APPSs_disagreement_implausibility_present;
  bool BPPC_implausibility_present;
  bool brake_shorted_or_opened_implausibility_present;
  bool APPSs_invalid_implausibility_present;
};

class Implausibilities {
 public:
  Implausibilities(VirtualTimerGroup& implausibility_group, Throttle& throttle, Brake& brake)
      : _implausibility_timers(implausibility_group), _throttle(throttle), _brake(brake) {};

  // Struct to store implausibility data
  ImplausibilitiesData implausibilities;
  // Check for implausibilities
  bool is_implausibility_present();

 private:
  // Timer group to store implausibility timers
  VirtualTimerGroup& _implausibility_timers;
  // Throttle
  Throttle& _throttle;
  // Brake
  Brake& _brake;

  // Init timer group
  void init_implausibility_timers();
  // Check for implausibilities
  void check_implausibilities();
  // Timer callbacks
  void set_is_APPSs_disagreement_implausibility_present_to_true();
  void set_is_brake_shorted_or_opened_implausibility_present_to_true();
  void set_APPSs_invalid_implausibility_present_to_true();
  // Check implausibilities
  void check_BPPC_implausibility();
  void check_brake_shorted_or_opened_implausibility();
  void check_APPSs_valid_implausibility();
  void check_APPSs_disagreement_implausibility();
  bool check_APPSs_validity() const;
};