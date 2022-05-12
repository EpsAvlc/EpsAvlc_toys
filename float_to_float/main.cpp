// CSAPP chapter 2.4 floating point num.

#include <bitset>
#include <cmath>
#include <iostream>

union ftof_util {
  float num_f;
  unsigned long num_u;
};

long double floatTofloat(const float num) {
  ftof_util util;
  util.num_f = num;
  std::bitset<32> bits(util.num_u);
  long double ret = 1;
  if (bits[31]) {
    ret = -1;
  }

  // std::cout << bits.to_string() << std::endl;

  bool normalized = false;
  uint8_t exp = 0;
  for (int i = 23; i <= 30; ++i) {
    if (bits[i] == true) {
      normalized = true;
      exp |= 1 << (i - 23);
    }
  }

  // std::bitset<8> exp_bits(exp);

  float frac = 0;
  uint32_t tmp = 0;
  for (int i = 0; i <= 22; ++i) {
    if (bits[i] == true) {
      tmp |= 1 << i;
    }
  }
  frac = static_cast<float>(tmp) / std::pow(2, 23);

  if (normalized) {
    int E = exp - 127;
    float M = frac + 1;
    ret *= M * std::pow(2, E);
  } else {
    float M = frac;
    ret *= frac;
  }
  return ret;
}

int main(int, char **) {
  float num = 12.34f;
  float res = floatTofloat(num);
  std::cout << "origin: " << num << std::endl;
  std::cout << "after: " << res << std::endl;
  num = 0.34f;
  res = floatTofloat(num);
  std::cout << "origin: " << num << std::endl;
  std::cout << "after: " << res << std::endl;
}
