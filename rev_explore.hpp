#pragma once 

#include <string>
#include <unordered_set>
#include "tables.hpp"

using namespace std;

void reverse_explore(vector<Agent> state, int ts, int target_ts, fix16_t sensing_range, int num_lanes, int num_obstacles, int max_ts, bool lc_only, bool dump_states);
void reverse_explore(vector<vector<Agent>> state, int ts, int target_ts, fix16_t sensing_range, int num_lanes, int num_obstacles, int max_ts, bool lc_only, bool dump_states);

extern int rev_explore_solutions;
