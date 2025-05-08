#include "lut_can.hpp"

void LUTCan::setLUTIDResponse(uint8_t id) { accel_lut_id_response = id; }

RXLUT LUTCan::processCAN() {
  std::vector<int16_t> xPairs;
  std::vector<float> yPairs;

  // add x values to vector
  xPairs.push_back(static_cast<int16_t>(this->x_zero));
  xPairs.push_back(static_cast<int16_t>(this->x_one));
  xPairs.push_back(static_cast<int16_t>(this->x_two));
  xPairs.push_back(static_cast<int16_t>(this->x_three));
  xPairs.push_back(static_cast<int16_t>(this->x_four));
  xPairs.push_back(static_cast<int16_t>(this->x_five));
  xPairs.push_back(static_cast<int16_t>(this->x_six));
  xPairs.push_back(static_cast<int16_t>(this->x_seven));
  xPairs.push_back(static_cast<int16_t>(this->x_eight));
  xPairs.push_back(static_cast<int16_t>(this->x_nine));
  xPairs.push_back(static_cast<int16_t>(this->x_ten));
  xPairs.push_back(static_cast<int16_t>(this->x_eleven));
  xPairs.push_back(static_cast<int16_t>(this->x_twelve));
  xPairs.push_back(static_cast<int16_t>(this->x_thirteen));
  xPairs.push_back(static_cast<int16_t>(this->x_fourteen));
  xPairs.push_back(static_cast<int16_t>(this->x_fifteen));
  xPairs.push_back(static_cast<int16_t>(this->x_sixteen));
  xPairs.push_back(static_cast<int16_t>(this->x_seventeen));
  xPairs.push_back(static_cast<int16_t>(this->x_eighteen));
  xPairs.push_back(static_cast<int16_t>(this->x_nineteen));
  xPairs.push_back(static_cast<int16_t>(this->x_twenty));
  xPairs.push_back(static_cast<int16_t>(this->x_twenty_one));
  xPairs.push_back(static_cast<int16_t>(this->x_twenty_two));
  xPairs.push_back(static_cast<int16_t>(this->x_twenty_three));
  xPairs.push_back(static_cast<int16_t>(this->x_twenty_four));
  xPairs.push_back(static_cast<int16_t>(this->x_twenty_five));
  xPairs.push_back(static_cast<int16_t>(this->x_twenty_six));
  xPairs.push_back(static_cast<int16_t>(this->x_twenty_seven));
  xPairs.push_back(static_cast<int16_t>(this->x_twenty_eight));
  xPairs.push_back(static_cast<int16_t>(this->x_twenty_nine));

  // Add y values to vector
  yPairs.push_back(static_cast<float>(this->y_zero));
  yPairs.push_back(static_cast<float>(this->y_one));
  yPairs.push_back(static_cast<float>(this->y_two));
  yPairs.push_back(static_cast<float>(this->y_three));
  yPairs.push_back(static_cast<float>(this->y_four));
  yPairs.push_back(static_cast<float>(this->y_five));
  yPairs.push_back(static_cast<float>(this->y_six));
  yPairs.push_back(static_cast<float>(this->y_seven));
  yPairs.push_back(static_cast<float>(this->y_eight));
  yPairs.push_back(static_cast<float>(this->y_nine));
  yPairs.push_back(static_cast<float>(this->y_ten));
  yPairs.push_back(static_cast<float>(this->y_eleven));
  yPairs.push_back(static_cast<float>(this->y_twelve));
  yPairs.push_back(static_cast<float>(this->y_thirteen));
  yPairs.push_back(static_cast<float>(this->y_fourteen));
  yPairs.push_back(static_cast<float>(this->y_fifteen));
  yPairs.push_back(static_cast<float>(this->y_sixteen));
  yPairs.push_back(static_cast<float>(this->y_seventeen));
  yPairs.push_back(static_cast<float>(this->y_eighteen));
  yPairs.push_back(static_cast<float>(this->y_nineteen));
  yPairs.push_back(static_cast<float>(this->y_twenty));
  yPairs.push_back(static_cast<float>(this->y_twenty_one));
  yPairs.push_back(static_cast<float>(this->y_twenty_two));
  yPairs.push_back(static_cast<float>(this->y_twenty_three));
  yPairs.push_back(static_cast<float>(this->y_twenty_four));
  yPairs.push_back(static_cast<float>(this->y_twenty_five));
  yPairs.push_back(static_cast<float>(this->y_twenty_six));
  yPairs.push_back(static_cast<float>(this->y_twenty_seven));
  yPairs.push_back(static_cast<float>(this->y_twenty_eight));
  yPairs.push_back(static_cast<float>(this->y_twenty_nine));

  RXLUT lut;
  lut.fileStatus = static_cast<FileStatus>(static_cast<uint8_t>(this->file_status));
  lut.interpType = static_cast<InterpType>(static_cast<uint8_t>(this->interp_type));
  lut.LUTId = static_cast<uint8_t>(this->lut_id);
  lut.numPairs = static_cast<uint8_t>(this->num_lut_pairs);
  std::map<int16_t, float> m;
  for (int i = 0; i < lut.numPairs; i++) {
    m.insert({xPairs.at(i), yPairs.at(i)});
  }

  lut.lut = m;
  return lut;
};