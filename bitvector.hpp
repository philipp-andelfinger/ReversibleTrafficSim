#pragma once

#include <vector>

using namespace std;

class BitVector {
public:
  int read(int bits, bool resize);
  void write(int max_val, int val);
  size_t size() { return vec.size(); };

private:
  vector<bool> vec;
};
