#include "implausibilities.hpp"

// Init timer group
void Implausibilities::init_implausibility_timers() {}
// Check for implausibilities
void Implausibilities::check_implausibilities() {}
// Timer callbacks
void Implausibilities::set_is_APPSs_disagreement_implausibility_present_to_true() {}
void Implausibilities::set_is_brake_shorted_or_opened_implausibility_present_to_true() {}
void Implausibilities::set_APPSs_invalid_implausibility_present_to_true() {}
// Check implausibilities
void Implausibilities::check_BPPC_implausibility() {}
void Implausibilities::check_brake_shorted_or_opened_implausibility() {}
void Implausibilities::check_APPSs_valid_implausibility() {}
void Implausibilities::check_APPSs_disagreement_implausibility() {}
bool Implausibilities::check_APPSs_validity() const {}