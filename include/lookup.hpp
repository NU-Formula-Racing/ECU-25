#pragma once
#include <cmath>
#include <map>

class Lookup {
 public:
  Lookup(const std::map<int16_t, float> table) : table(table) {}

  float lookup_val(int16_t key) const;

 private:
  const std::map<int16_t, float> table;
};