#include <vector>
#include <numeric>
#include "bitvector.hpp"
#include "util.hpp"

using namespace std;

int BitVector::read(int num_bits, bool resize)
{
  if(!num_bits || !size())
    return 0;

  int r = accumulate(vec.end() - num_bits, vec.end(), 0,
                     [](int a, int b) { return (a << 1) + b; });

  if(resize)
    vec.resize(vec.size() - num_bits);

  return r;
}

void BitVector::write(int num_bits, int val)
{
  if(!num_bits)
    return;

  int val_bits = ceil_log2(val);

  for(int i = 0; i < num_bits - val_bits; i++)
    vec.push_back(0);

  for(int i = val_bits - 1; i >= 0; i--) {
    vec.push_back((val >> i) & 1);
  }
}
