#include "lookup.hpp"

#include <cmath>
#include <map>

float Lookup::lookup_val(int16_t key) {
  auto it = table.lower_bound(key);
  // if key is smaller than the smallest key, return the first value
  if (it == table.begin()) {
    return it->second;
  }
  // if key is larger than the largest key, return the last value
  if (it == table.end()) {
    return std::prev(it)->second;
  }
  // if key is exactly found in the LUT, return its value
  if (it->first == key) {
    return it->second;
  }
  // if we're here, key is btwn 2 entries in the LUT, interpolate
  auto upper = it;
  auto lower = std::prev(it);

  return lower->second + (upper->second - lower->second) * static_cast<float>(key - lower->first) /
                             static_cast<float>(upper->first - lower->first);
}