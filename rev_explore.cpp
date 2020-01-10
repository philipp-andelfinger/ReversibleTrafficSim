#include <iostream>
#include <sstream>
#include "tables.hpp"
#include "main.hpp"
#include "lc.hpp"
#include "cf.hpp" // for ipow

int rev_explore_solutions = 0;

static const bool debug = false;

void choose(int num_vehicles, int num_lcs, vector<pair<uint8_t, int8_t>> &v, int start_pos, vector<vector<pair<uint8_t, int8_t>>> &lc_decisions)
{
  int decision_mapping[] = {-1, 1};
  for(int i = start_pos; i <= num_vehicles - num_lcs; i++) {
    for(uint8_t decision_index = 0; decision_index <= 1; decision_index++) {
      pair<uint8_t, int8_t> dec(i, decision_mapping[decision_index]);
      v.push_back(dec);

      if(num_lcs == 1) {
        lc_decisions.push_back(v);
      } else {
        choose(num_vehicles, num_lcs - 1, v, i + 1, lc_decisions);
      }

      v.pop_back();
    }
  }
}

// this gives us a valid previous state starting from curr_state
// in the first call at each time stamp and branch, set initialize to true
bool get_prev_state_cf(vector<Agent> &curr_state, vector<Agent> &prev_state, vector<int> &disambig_indexes, fix16_t sensing_range, bool initialize)
{
  static const fix16_t veh_length = fix16_from_str("4.5");
  int aidx = curr_state.size() - 1; // the agent index to try to increment first

  do {
    if(initialize) {
      aidx = 1;
      initialize = false;
    } else {
      if(debug)
        cerr << "curr_state.size(): " << curr_state.size() << ", prev_state.size(): " << prev_state.size() << endl;
 
      for(; aidx > 0; aidx--) {
        if(curr_state[aidx].lane != curr_state[aidx - 1].lane || !curr_state[aidx].id)
          continue;

        fix16_t old_p = curr_state[aidx].p - p_change[curr_state[aidx].v];
        fix16_t old_p_delta = prev_state[aidx - 1].p - old_p - veh_length;
        if(old_p_delta > sensing_range)
          old_p_delta = INT32_MAX;

        Bw_key bw_key = { curr_state[aidx].v, prev_state[aidx - 1].v, old_p_delta, 0 };
        int num_ins = num_fw_keys[bw_key];

        if(debug)
          cerr << "aidx is " << aidx << ", getting num_ins for agent " << curr_state[aidx].id << ", old_p_ahead: " << prev_state[aidx - 1].p << ", " << bw_key.new_v << ", " << bw_key.old_v_ahead << ", " << bw_key.old_p_delta << ", " << bw_key.index << ": " << num_fw_keys[bw_key] << endl;
        if(!num_ins)
          return false;

        disambig_indexes[aidx] = (disambig_indexes[aidx] + 1) % num_ins;

        if(debug) {
          cerr << "incremented for aidx " << aidx << ", now at " << disambig_indexes[aidx] << endl;
          cerr << "disambig_indexes for " << aidx << " is now " << disambig_indexes[aidx] << ", num_ins: " << num_ins << endl;
        }

        if(disambig_indexes[aidx] != 0) {
          for(int aidx_ = aidx + 1; aidx_ < curr_state.size(); aidx_++) {
            disambig_indexes[aidx_] = 0;
            if(debug)
              cerr << "disambig_indexes for " << aidx_ << " is now " << disambig_indexes[aidx_] << endl;
          }
          break;
        }
      }

      if(aidx == 0) {
        if(debug)
          cerr << "reached end of state" << endl;
        return false; // no more reachable states at this ts
      }
    }

    for(; aidx < prev_state.size(); aidx++) {
      if(curr_state[aidx].lane != curr_state[aidx - 1].lane || !curr_state[aidx].id)
        continue;

      fix16_t old_p = curr_state[aidx].p - p_change[curr_state[aidx].v];
      fix16_t old_p_delta = prev_state[aidx - 1].p - old_p - veh_length;
      if(old_p_delta > sensing_range)
        old_p_delta = INT32_MAX;

      Bw_key bw_key = { curr_state[aidx].v, prev_state[aidx - 1].v, old_p_delta, 0 };
      int num_ins = num_fw_keys[bw_key];
      if(debug) {
        cerr << "aidx is " << aidx << ", checking ins for agent " << curr_state[aidx].id << ", new_v: " << bw_key.new_v << ", old_v_ahead: " << bw_key.old_v_ahead << ", old_p_delta: " << bw_key.old_p_delta << ", index: " << bw_key.index << endl;
        cerr << "old_v_ahead: " << prev_state[aidx - 1].v << ", old_p_ahead: " << prev_state[aidx - 1].p << ", old_p: " << old_p << ", " << p_change[curr_state[aidx].v] << endl;
        cerr << "there are " << num_fw_keys[bw_key] << " ins" << endl;
      }

      if(!num_ins) {
        aidx--;
        break;
      }

      bw_key.index = disambig_indexes[aidx];
      auto it = bw_table.find(bw_key);
      if(it == bw_table.end()) {
        cerr << "no v for bw key " << bw_key.new_v << ", " << bw_key.old_v_ahead << ", " << bw_key.old_p_delta << ", " << bw_key.index << endl;

        exit(1);
      }
 
      if(debug)
        cerr << it->second << " <- " << bw_key.new_v << ", " << bw_key.old_v_ahead << ", " << bw_key.old_p_delta << ", " << bw_key.index << endl;

      prev_state[aidx].v = it->second;
      prev_state[aidx].p = old_p;
    }

    if(debug)
      cerr << "aidx is " << aidx << ", disambig_index: " << disambig_indexes[aidx] << ", initialize is " << initialize << endl;
  } while(aidx != prev_state.size());

  for(int aidx = 0; aidx < prev_state.size() - 1; aidx++) {
    Agent &leader = prev_state[aidx];
    Agent &follower = prev_state[aidx + 1];
  }

  return true;
}

bool get_prev_state_lc(vector<Agent> &prev_state_cf, vector<vector<Agent>> &prev_state_cf_split, vector<Agent> &prev_state_lc, vector<pair<uint8_t, int8_t>> &lc_decision, int num_lanes)
{
  fix16_t v_max = fix16_from_int(20);

  int lc_dec_pos = 0;
  int lc_dec_mobile_vehicle = 0;
  for(int i = 0; i < prev_state_cf.size(); i++) {
    prev_state_lc[i] = prev_state_cf[i];
    if(prev_state_lc[i].lane < 0 || prev_state_lc[i].lane > num_lanes - 1) {
      cerr << "BUG in get_prev_state_lc! agent " << prev_state_lc[i].id << " claims to be on lane " << prev_state_lc[i].lane << endl;
      exit(1);
    }
  }

  for(int i = 0; i < prev_state_cf.size() && lc_dec_pos < lc_decision.size(); i++) {
    if(!prev_state_cf[i].id)
      continue;

    if(lc_decision[lc_dec_pos].first != lc_dec_mobile_vehicle) {
      lc_dec_mobile_vehicle++;
      continue;
    }

    int lc_dec = lc_decision[lc_dec_pos].second;
    lc_dec_pos++;
    lc_dec_mobile_vehicle++;

    if(debug)
      cerr << "trying " << prev_state_cf[i].id << " on lane " << prev_state_cf[i].lane - lc_dec << " instead of " << prev_state_cf[i].lane << endl;
    
    prev_state_lc[i].lane = prev_state_cf[i].lane - lc_dec;
    if(prev_state_lc[i].lane < 0 || prev_state_lc[i].lane > num_lanes - 1) {
      return false;
    }

  }

  if(lc_dec_pos != lc_decision.size()) {
    cerr << "BUG! lc_dec_pos is " << lc_dec_pos << ", lc_decision.size() is " << lc_decision.size() << endl;
    exit(1);
  }

 
  vector<vector<Agent>> prev_state_lc_split(num_lanes);

  if(debug)
    dump_state(prev_state_lc, "prev_state_lc before sorting");
  sort(prev_state_lc.begin(), prev_state_lc.end());
  reverse(prev_state_lc.begin(), prev_state_lc.end());
  if(debug)
    dump_state(prev_state_lc, "prev_state_lc after sorting");
  
  split_state_by_lane(prev_state_lc, prev_state_lc_split, num_lanes);
  if(debug)
    dump_state(prev_state_lc_split, "prev_state_lc_split");


  static vector<vector<Agent>> cand_prev_state_cf(num_lanes);
  for(int lidx = 0; lidx < num_lanes; lidx++)
    cand_prev_state_cf[lidx].clear();


  for(int lidx = 0; lidx < num_lanes; lidx++) {

    cand_prev_state_cf[lidx].push_back(prev_state_lc_split[lidx][0]);
    for(int aidx = 1; aidx < prev_state_lc_split[lidx].size(); aidx++) {
      int targetLane = lidx;
      Agent a = prev_state_lc_split[lidx][aidx];

      if(a.id) {
        int lc_result = lc(prev_state_lc_split, lidx, aidx, v_max);
    
        targetLane += lc_result;

        a.lane = targetLane;
      }

      cand_prev_state_cf[targetLane].push_back(a);
    }

  }


  for(int lidx = 0; lidx < num_lanes; lidx++) {
    sort(cand_prev_state_cf[lidx].begin(), cand_prev_state_cf[lidx].end());
    reverse(cand_prev_state_cf[lidx].begin(), cand_prev_state_cf[lidx].end());
  }

  if(debug) {
    dump_state(cand_prev_state_cf, "cand_prev_state_cf");
    dump_state(prev_state_cf_split, "prev_state_cf_split");
  }


  bool r = true;
  for(int lidx = 0; lidx < num_lanes; lidx++) {
    if(cand_prev_state_cf[lidx].size() != prev_state_cf_split[lidx].size()) {
      if(debug)
        cerr << "different numbers of agents on lane " << lidx << endl;
      return false;
    } else {

      for(int aidx = 0; aidx < cand_prev_state_cf[lidx].size(); aidx++) {
        if(debug)
          cerr << cand_prev_state_cf[lidx][aidx].id << ", " << cand_prev_state_cf[lidx][aidx].lane << " vs " << prev_state_cf_split[lidx][aidx].id << ", " << prev_state_cf_split[lidx][aidx].lane << endl;

        if(cand_prev_state_cf[lidx][aidx] != prev_state_cf_split[lidx][aidx]) {
          if(debug)
            cerr << "different agents on the lane" << endl;
          return false;
        }
      }
    }
  }

  return true;

}

void reverse_explore(vector<Agent> state, int ts, int target_ts, fix16_t sensing_range, int num_lanes, int num_obstacles, int max_ts, bool lc_only, bool dump_states)
{

  const bool detailed_statistics = true;
  const int detailed_statistics_period = 10000000;
  const int forward_dump_period = 1;

  const int num_vehicles = state.size();

  const int num_mobile_vehicles = num_vehicles - num_lanes - num_obstacles;

  static uint64_t num_outer_states_lc = 0;
  static uint64_t num_outer_states_cf = 0;

  static int old_rev_explore_solutions = 0;

  static int min_ts = INT_MAX;
  static map<int, uint64_t> total_cf_at_ts, successful_cf_at_ts, total_lc_at_ts, successful_lc_at_ts;
 
  if(detailed_statistics_period && ts == max_ts) {
    total_cf_at_ts.clear();
    successful_cf_at_ts.clear();
    total_lc_at_ts.clear();
    successful_lc_at_ts.clear();
  }

  static time_t start_secs = time(NULL);

  if(ts == target_ts) {
    if(dump_states)
      dump_state(state, "starting state");

    rev_explore_solutions++;

    if(!debug)
      return;

    static uint64_t forward_dump_count = 0;

    if(!(rev_explore_solutions % forward_dump_period)) {
      string path = "traces/";
      string filename = to_string(forward_dump_count);
      forward_dump_count++;

      filename.insert(0, 20 - filename.length(), '0');
      filename.append(".csv");
      filename = path + filename;

      vector<vector<Agent>> split_state(num_lanes);
      for(int lidx = 0; lidx < num_lanes; lidx++) {
        split_state[lidx].reserve(num_vehicles);
      }

      split_state_by_lane(state, split_state, num_lanes);

      int final_step; // unused
      simulate_forward(split_state, sensing_range, max_ts, final_step, filename);

    }

    return;
  }

  bool initialize_cf = true;
  vector<Agent> prev_state_cf(state);
  vector<int> disambig_indexes_cf(num_vehicles);
  while(true) {

    if(!lc_only) {
      bool r = get_prev_state_cf(state, prev_state_cf, disambig_indexes_cf, sensing_range, initialize_cf);
      initialize_cf = false;

      num_outer_states_cf++;

      if(detailed_statistics)
        total_cf_at_ts[ts - 1]++;

      if(!r) { // no further valid cf states at this time stamp
        break;
      }

      if(detailed_statistics)
        successful_cf_at_ts[ts - 1]++;
    }

    // now, prev_state is a valid state prior to car-following
    // next, we need a valid prev state prior to lane-changing

    vector<Agent> prev_state_lc(prev_state_cf);
    vector<vector<Agent>> prev_state_cf_split(num_lanes);
    for(int lidx = 0; lidx < num_lanes; lidx++)
      prev_state_cf_split[lidx].reserve(num_vehicles);

    split_state_by_lane(prev_state_cf, prev_state_cf_split, num_lanes);

    vector<vector<pair<uint8_t, int8_t>>> lc_decisions;
    vector<pair<uint8_t, int8_t>> v;

    lc_decisions.push_back(v);

    for(int num_lcs = 1; num_lcs <= num_mobile_vehicles; num_lcs++) {
      if(debug)
        cerr << ts << ", trying lc of " << num_lcs << " agents" << endl;
  
      choose(num_mobile_vehicles, num_lcs, v, 0, lc_decisions);
      v.clear();

      if(debug)
        cerr << "lc_decisions.size(): " << lc_decisions.size() << endl;

      for(int lc_decision = 0; lc_decision < lc_decisions.size(); lc_decision++) {
        if(debug)
          cerr << ts << ", trying lc_decision " << lc_decision << " out of " << lc_decisions.size() << endl;

        bool r = get_prev_state_lc(prev_state_cf, prev_state_cf_split, prev_state_lc, lc_decisions[lc_decision], num_lanes);
        if(debug)
          cerr << "got lc result: " << r << endl;

        if(detailed_statistics)
          total_lc_at_ts[ts - 1]++;

        num_outer_states_lc++;
        if(!(num_outer_states_lc % detailed_statistics_period) && rev_explore_solutions != old_rev_explore_solutions) {

          time_t elapsed_secs = time(NULL) - start_secs;
          if(elapsed_secs) {
            int cf_per_sec = num_outer_states_cf / elapsed_secs;
            int lc_per_sec = num_outer_states_lc / elapsed_secs;
         
            cerr << "valid solutions found: " << rev_explore_solutions << endl;
            cerr << "explored states total cf: " << num_outer_states_cf << ", lc: " << num_outer_states_lc << endl;
            cerr << "per sec wall-clock time: " << cf_per_sec << ", " << lc_per_sec << endl;

            if(detailed_statistics) {
              cerr << "- ts, total cf, successful cf, total lc, successful lc" << endl;
              if(total_cf_at_ts.size()) {
                for(auto it = total_cf_at_ts.rbegin(); it != total_cf_at_ts.rend(); it++) {
                  cerr << "- " << it->first << ", " << it->second << ", " << successful_cf_at_ts[it->first] << ", " << total_lc_at_ts[it->first] << ", " << successful_lc_at_ts[it->first] << endl;
                }
              } else {
                for(auto it = total_lc_at_ts.rbegin(); it != total_lc_at_ts.rend(); it++) {
                  cerr << it->first << ", " << it->second << ", " << successful_cf_at_ts[it->first] << ", " << total_lc_at_ts[it->first] << ", " << successful_lc_at_ts[it->first] << endl;
                }
              }
            }
          }
        }

        if(!r) // the current combination does not represent a valid state
          continue;

        if(detailed_statistics)
          successful_lc_at_ts[ts - 1]++;


        if(ts < min_ts) {
          cerr << "lowest ts: " << ts << endl;
          min_ts = ts;
        }

        if(debug) {
          // check whether previous state actually leads into the current state

          vector<vector<Agent>> split_prev_state_lc(num_lanes);
          split_state_by_lane(prev_state_lc, split_prev_state_lc, num_lanes);
          vector<vector<Agent>> split_state(num_lanes);
          split_state_by_lane(state, split_state, num_lanes);

          int final_ts;
          simulate_forward(split_prev_state_lc, sensing_range, 1, final_ts, "");

          for(int lidx = 0; lidx < num_lanes; lidx++) {
            if(split_prev_state_lc[lidx].size() != split_state[lidx].size()) {
              cerr << "number of vehicles on a lane of 'prev_state_lc' are different than of 'state'" << endl;
              exit(1);
            }
            for(int aidx = 0; aidx < split_prev_state_lc[lidx].size(); aidx++) {
              if(split_prev_state_lc[lidx][aidx] != split_state[lidx][aidx]) {
                cerr << "'prev_state_lc' does not lead into 'state'" << endl;
                exit(1);
              }
            }
          }
        }

        reverse_explore(prev_state_lc, ts - 1, target_ts, sensing_range, num_lanes, num_obstacles, max_ts, false, dump_states);
      }
     
      lc_decisions.clear();

    }

    if(lc_only)
      break;

  }

  if(ts == max_ts && rev_explore_solutions != old_rev_explore_solutions) {
    time_t elapsed_secs = max((int)1, (int)(time(NULL) - start_secs));
    if(elapsed_secs) {
      int cf_per_sec = num_outer_states_cf / elapsed_secs;
      int lc_per_sec = num_outer_states_lc / elapsed_secs;
   
      cerr << "valid solutions found: " << rev_explore_solutions << endl;
      cerr << "explored states total cf: " << num_outer_states_cf << ", lc: " << num_outer_states_lc << endl;
      cerr << "per sec wall-clock time: " << cf_per_sec << ", " << lc_per_sec << endl;
  
      if(detailed_statistics) {
        cerr << "- ts, total cf, successful cf, total lc, successful lc" << endl;
        if(total_cf_at_ts.size()) {
          for(auto it = total_cf_at_ts.rbegin(); it != total_cf_at_ts.rend(); it++) {
            cerr << "- " << it->first << ", " << it->second << ", " << successful_cf_at_ts[it->first] << ", " << total_lc_at_ts[it->first] << ", " << successful_lc_at_ts[it->first] << endl;
          }
        } else {
          for(auto it = total_lc_at_ts.rbegin(); it != total_lc_at_ts.rend(); it++) {
            cerr << it->first << ", " << it->second << ", " << successful_cf_at_ts[it->first] << ", " << total_lc_at_ts[it->first] << ", " << successful_lc_at_ts[it->first] << endl;
          }
        }
      }
    }
    old_rev_explore_solutions = rev_explore_solutions;
  }


}

void reverse_explore(vector<vector<Agent>> state, int ts, int target_ts, fix16_t sensing_range, int num_lanes, int num_obstacles, int max_ts, bool lc_only, bool dump_states)
{
  vector<Agent> linear_state;
  flatten_state(state, linear_state);
  reverse_explore(linear_state, ts, target_ts, sensing_range, num_lanes, num_obstacles, max_ts, lc_only, dump_states);
}
