#pragma once
#include <cmath>
#include <map>

class Lookup {
 public:
  Lookup(std::map<int16_t, float> table) : table(table) {}

  float lookup_val(int16_t key);

 private:
  std::map<int16_t, float> table;
};