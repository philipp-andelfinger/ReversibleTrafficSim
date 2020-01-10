#pragma once

#include <unordered_map>
#include <unordered_set>
#include "util.hpp"

using namespace std;

void create_tables(fix16_t sensing_range, fix16_t v_step, fix16_t p_step, double dt_dbl, bool dump_input_transitions);

extern unordered_map<Bw_key, fix16_t> bw_table;
extern unordered_map<Bw_key, uint32_t> num_fw_keys;
extern unordered_map<fix16_t, fix16_t> p_change;

Fw_val fw_table_get(fix16_t v, fix16_t v_ahead, fix16_t p);
