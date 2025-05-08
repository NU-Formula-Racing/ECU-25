#pragma once
#ifndef LUT_CAN
#define LUT_CAN
#include <map>

#include "can_interface.h"
#include "esp_can.h"
#include "virtualTimer.h"

enum class FileStatus : uint8_t {
  FILE_PRESENT_AND_VALID = 0,
  FILE_NOT_PRESENT = 1,
  INVALID_KEY_PHRASE = 2,
  LOGGING_ERROR = 3,
  FILE_OPEN_ERROR = 4
};

enum class InterpType : uint8_t { LINEAR = 0, SMOOTH_STEP = 1 };

struct RXLUT {
  FileStatus fileStatus;
  uint8_t numPairs;
  InterpType interpType;
  uint8_t LUTId;
  std::map<int16_t, float> lut;
};

class LUTCan {
 public:
  LUTCan(ICAN& can_interface, VirtualTimerGroup& timers)
      : can_bus(can_interface), timers(timers) {};

  RXLUT processCAN();
  std::map<int16_t, float> getLUT();
  FileStatus getFileStatus();
  InterpType getInterpType();
  uint8_t getLUTid();
  void setLUTIDResponse(uint8_t id);

 private:
  ICAN& can_bus;
  VirtualTimerGroup& timers;

  MakeUnsignedCANSignal(uint8_t, 0, 8, 1, 0) accel_lut_id_response {};
  CANTXMessage<1> ecu_lut_response{can_bus, 0x20A, 1, 100, accel_lut_id_response};

  MakeUnsignedCANSignal(uint8_t, 0, 8, 1.0, 0.0) file_status {};
  MakeUnsignedCANSignal(uint8_t, 8, 8, 1.0, 0.0) num_lut_pairs {};
  MakeUnsignedCANSignal(uint8_t, 16, 8, 1.0, 0.0) interp_type {};
  MakeUnsignedCANSignal(uint8_t, 24, 8, 1.0, 0.0) lut_id {};

  CANRXMessage<4> daq_lut_metadata{can_bus, 0x2B0, file_status, num_lut_pairs, interp_type, lut_id};

  MakeSignedCANSignal(int16_t, 0, 16, 1.0, 0.0) x_zero {};
  MakeSignedCANSignal(float, 16, 16, 1.0, 0.0) y_zero {};
  MakeSignedCANSignal(int16_t, 32, 16, 1.0, 0.0) x_one {};
  MakeSignedCANSignal(float, 48, 16, 1.0, 0.0) y_one {};

  CANRXMessage<4> daq_lut_pair_zero_one{can_bus, 0x2B1, x_zero, y_zero, x_one, y_one};

  MakeSignedCANSignal(int16_t, 0, 16, 1.0, 0.0) x_two {};
  MakeSignedCANSignal(float, 16, 16, 1.0, 0.0) y_two {};
  MakeSignedCANSignal(int16_t, 32, 16, 1.0, 0.0) x_three {};
  MakeSignedCANSignal(float, 48, 16, 1.0, 0.0) y_three {};

  CANRXMessage<4> daq_lut_pair_two_three{can_bus, 0x2B2, x_two, y_two, x_three, y_three};

  MakeSignedCANSignal(int16_t, 0, 16, 1.0, 0.0) x_four {};
  MakeSignedCANSignal(float, 16, 16, 1.0, 0.0) y_four {};
  MakeSignedCANSignal(int16_t, 32, 16, 1.0, 0.0) x_five {};
  MakeSignedCANSignal(float, 48, 16, 1.0, 0.0) y_five {};

  CANRXMessage<4> daq_lut_pair_four_five{can_bus, 0x2B3, x_four, y_four, x_five, y_five};

  MakeSignedCANSignal(int16_t, 0, 16, 1.0, 0.0) x_six {};
  MakeSignedCANSignal(float, 16, 16, 1.0, 0.0) y_six {};
  MakeSignedCANSignal(int16_t, 32, 16, 1.0, 0.0) x_seven {};
  MakeSignedCANSignal(float, 48, 16, 1.0, 0.0) y_seven {};

  CANRXMessage<4> daq_lut_pair_six_seven{can_bus, 0x2B4, x_six, y_six, x_seven, y_seven};

  MakeSignedCANSignal(int16_t, 0, 16, 1.0, 0.0) x_eight {};
  MakeSignedCANSignal(float, 16, 16, 1.0, 0.0) y_eight {};
  MakeSignedCANSignal(int16_t, 32, 16, 1.0, 0.0) x_nine {};
  MakeSignedCANSignal(float, 48, 16, 1.0, 0.0) y_nine {};

  CANRXMessage<4> daq_lut_pair_eight_nine{can_bus, 0x2B5, x_eight, y_eight, x_nine, y_nine};

  MakeSignedCANSignal(int16_t, 0, 16, 1.0, 0.0) x_ten {};
  MakeSignedCANSignal(float, 16, 16, 1.0, 0.0) y_ten {};
  MakeSignedCANSignal(int16_t, 32, 16, 1.0, 0.0) x_eleven {};
  MakeSignedCANSignal(float, 48, 16, 1.0, 0.0) y_eleven {};

  CANRXMessage<4> daq_lut_pair_ten_eleven{can_bus, 0x2B6, x_ten, y_ten, x_eleven, y_eleven};

  MakeSignedCANSignal(int16_t, 0, 16, 1.0, 0.0) x_twelve {};
  MakeSignedCANSignal(float, 16, 16, 1.0, 0.0) y_twelve {};
  MakeSignedCANSignal(int16_t, 32, 16, 1.0, 0.0) x_thirteen {};
  MakeSignedCANSignal(float, 48, 16, 1.0, 0.0) y_thirteen {};

  CANRXMessage<4> daq_lut_pair_twelve_thirteen{can_bus,  0x2B7,      x_twelve,
                                               y_twelve, x_thirteen, y_thirteen};

  MakeSignedCANSignal(int16_t, 0, 16, 1.0, 0.0) x_fourteen {};
  MakeSignedCANSignal(float, 16, 16, 1.0, 0.0) y_fourteen {};
  MakeSignedCANSignal(int16_t, 32, 16, 1.0, 0.0) x_fifteen {};
  MakeSignedCANSignal(float, 48, 16, 1.0, 0.0) y_fifteen {};

  CANRXMessage<4> daq_lut_pair_thirteen_fourteen{can_bus,    0x2B8,     x_fourteen,
                                                 y_fourteen, x_fifteen, y_fifteen};

  MakeSignedCANSignal(int16_t, 0, 16, 1.0, 0.0) x_sixteen {};
  MakeSignedCANSignal(float, 16, 16, 1.0, 0.0) y_sixteen {};
  MakeSignedCANSignal(int16_t, 32, 16, 1.0, 0.0) x_seventeen {};
  MakeSignedCANSignal(float, 48, 16, 1.0, 0.0) y_seventeen {};

  CANRXMessage<4> daq_lut_pair_sixteen_seventeen{can_bus,   0x2B9,       x_sixteen,
                                                 y_sixteen, x_seventeen, y_seventeen};

  MakeSignedCANSignal(int16_t, 0, 16, 1.0, 0.0) x_eighteen {};
  MakeSignedCANSignal(float, 16, 16, 1.0, 0.0) y_eighteen {};
  MakeSignedCANSignal(int16_t, 32, 16, 1.0, 0.0) x_nineteen {};
  MakeSignedCANSignal(float, 48, 16, 1.0, 0.0) y_nineteen {};

  CANRXMessage<4> daq_lut_pair_eighteen_nineteen{can_bus,    0x2BA,      x_eighteen,
                                                 y_eighteen, x_nineteen, y_nineteen};

  MakeSignedCANSignal(int16_t, 0, 16, 1.0, 0.0) x_twenty {};
  MakeSignedCANSignal(float, 16, 16, 1.0, 0.0) y_twenty {};
  MakeSignedCANSignal(int16_t, 32, 16, 1.0, 0.0) x_twenty_one {};
  MakeSignedCANSignal(float, 48, 16, 1.0, 0.0) y_twenty_one {};

  CANRXMessage<4> daq_lut_pair_twenty_twenty_one{can_bus,  0x2BB,        x_twenty,
                                                 y_twenty, x_twenty_one, y_twenty_one};

  MakeSignedCANSignal(int16_t, 0, 16, 1.0, 0.0) x_twenty_two {};
  MakeSignedCANSignal(float, 16, 16, 1.0, 0.0) y_twenty_two {};
  MakeSignedCANSignal(int16_t, 32, 16, 1.0, 0.0) x_twenty_three {};
  MakeSignedCANSignal(float, 48, 16, 1.0, 0.0) y_twenty_three {};

  CANRXMessage<4> daq_lut_pair_twenty_two_twenty_three{
      can_bus, 0x2BC, x_twenty_two, y_twenty_two, x_twenty_three, y_twenty_three};

  MakeSignedCANSignal(int16_t, 0, 16, 1.0, 0.0) x_twenty_four {};
  MakeSignedCANSignal(float, 16, 16, 1.0, 0.0) y_twenty_four {};
  MakeSignedCANSignal(int16_t, 32, 16, 1.0, 0.0) x_twenty_five {};
  MakeSignedCANSignal(float, 48, 16, 1.0, 0.0) y_twenty_five {};

  CANRXMessage<4> daq_lut_pair_twenty_four_twenty_five{can_bus,       0x2BD,         x_twenty_four,
                                                       y_twenty_four, x_twenty_five, y_twenty_five};

  MakeSignedCANSignal(int16_t, 0, 16, 1.0, 0.0) x_twenty_six {};
  MakeSignedCANSignal(float, 16, 16, 1.0, 0.0) y_twenty_six {};
  MakeSignedCANSignal(int16_t, 32, 16, 1.0, 0.0) x_twenty_seven {};
  MakeSignedCANSignal(float, 48, 16, 1.0, 0.0) y_twenty_seven {};

  CANRXMessage<4> daq_lut_pair_twenty_six_twenty_seven{
      can_bus, 0x2BE, x_twenty_six, y_twenty_six, x_twenty_seven, y_twenty_seven};

  MakeSignedCANSignal(int16_t, 0, 16, 1.0, 0.0) x_twenty_eight {};
  MakeSignedCANSignal(float, 16, 16, 1.0, 0.0) y_twenty_eight {};
  MakeSignedCANSignal(int16_t, 32, 16, 1.0, 0.0) x_twenty_nine {};
  MakeSignedCANSignal(float, 48, 16, 1.0, 0.0) y_twenty_nine {};

  CANRXMessage<4> daq_lut_pair_twenty_eight_twenty_nine{
      can_bus, 0x2BF, x_twenty_eight, y_twenty_eight, x_twenty_nine, y_twenty_nine};
};

#endif