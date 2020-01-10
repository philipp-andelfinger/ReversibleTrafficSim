#include <math.h>
#include <iostream>
#include "util.hpp"

using namespace std;

double idm(double v, double v_delta, double d_delta)
{
  const double sqrtab = 3.0;
  const double s0 = 1.5;
  const double T = 1.5;
  const double a = 3;
  const double v_desired = 20.0;

  const double sensing_range = 40.0;

  if(d_delta > sensing_range)
    d_delta = INFINITY;

  return (-((a * ipow((s0 + T * v + (v * (-v_delta))/(2 * sqrtab)), 2))/ipow((d_delta), 2)) + a * (1 - (ipow(v, 4))/(ipow(v_desired, 4))));
}
