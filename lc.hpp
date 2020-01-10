#include <fixmath/fixmath.h>
#include "util.hpp"

void findClosestVehicles(const vector<Agent> &lane, fix16_t position, int veh_indexes[2]);
int lc(const vector<vector<Agent>> &state, int veh_lane, int veh_index, fix16_t v_max);
