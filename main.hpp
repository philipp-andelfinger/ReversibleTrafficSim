#pragma once

#include <fixmath/fixmath.h>
#include <unordered_map>

bool simulate_forward(vector<vector<Agent>> &state, fix16_t sensing_range, int steps, int &final_step, string out_filename);
