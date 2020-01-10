#!/usr/bin/python3

import math
import sys

def g_lc(b_e):
  return p_lc * (2 + b_e) + 1.0 / 2**b_e

def calc_b_e(p_lc):
  r = max(1, math.log2(math.log(2) / p_lc))

  b_e_floor = g_lc(math.floor(r))
  b_e_ceil = g_lc(math.ceil(r))
  
  if b_e_floor < b_e_ceil:
    return math.floor(r)
  else:
    return math.ceil(r)

print("p_lc, b_e, l_e")
inc = 0.0001
for p_lc in [x * inc for x in range(1, int(1.0 / inc))]:
  if sys.argv[1] == "optimal":
    b_e = calc_b_e(p_lc)
  else:
    b_e = math.log2(int(sys.argv[1]))

  print(str(p_lc) + ", " + str(b_e) + ", " + str(2**b_e) + ", " + str(g_lc(b_e)))
