#include <unordered_map>
#include <fixmath/fixmath.h>
#include <fstream>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include "bitvector.hpp"
#include "tables.hpp"
#include "lc.hpp"
#include "util.hpp"
#include "main.hpp"
#include "rev_explore.hpp"

using namespace std;
namespace pt = boost::posix_time;

const bool print_states = false;
const bool print_garbage_bit_statistics = true;

const int num_lanes = 3;

const int epoch_bits = 2;
const int epoch_length = ipow(2, epoch_bits);

static const bool debug = false;

bool simulate_forward(vector<vector<Agent>> &state, fix16_t sensing_range, int steps, int &final_step, string out_filename = "")
{
  static const fix16_t veh_length = fix16_from_str("4.5");
  const fix16_t v_max = fix16_from_int(20);
  ofstream out_file;
  if(out_filename.compare("")) {
    out_file.open(out_filename);
  }
  ostream &out_stream = out_file.is_open() ? out_file : cout;

  int ts = final_step = 0;
  for(; ts < steps; ts++, final_step++) {

    if(out_filename.compare("") || print_states) {
      for(int lidx = 0; lidx < state.size(); lidx++) {
        for(int aidx = 0; aidx < state[lidx].size(); aidx++) {
          out_stream << lidx << ", " << state[lidx][aidx].id << ", " << ts << ", " << state[lidx][aidx].v << ", " << state[lidx][aidx].p << endl;
        }
      }
    }

    vector<vector<Agent>> new_state(state.size());

    // insert epoch markers for all mobile vehicles
    if(ts && !(ts % epoch_length)) {
      for(int lidx = 0; lidx < state.size(); lidx++) {
        for(int aidx = 0; aidx < state[lidx].size(); aidx++) {
          Agent &a = state[lidx][aidx];
          if(!a.id)
            continue;
          
          a.garbage_bits_lc->write(1, 0);
        }
      }
    }

    for(int lidx = 0; lidx < state.size(); lidx++) {
      new_state[lidx].push_back(state[lidx][0]);
      for(int aidx = 1; aidx < state[lidx].size(); aidx++) {
        Agent &a = state[lidx][aidx];
        if(!a.id) {
          new_state[lidx].push_back(a);
          continue;
        }

        int lc_result = lc(state, lidx, aidx, v_max);

        if(lc_result) {
          if(debug)
            cerr << "lane change at ts " << ts << " by vehicle " << state[lidx][aidx].id << endl;

          if(a.garbage_bits_lc) {
            bool lc_bit = (lc_result > 0); // 0 for left, 1 for right

            a.garbage_bits_lc->write(1, lc_bit);
            a.garbage_bits_lc->write(epoch_bits, ts % epoch_length); // time step in epoch
            a.garbage_bits_lc->write(1, 1); // mark actual lane change

            if(debug)
              cerr << ts << ", " << ts % epoch_length << ", agent " << a.id << " changing to the lane on the " << (!lc_bit ? "left" : "right") << ", i.e., from " << lidx << " to " << lidx + lc_result << ", garbage bits: " << a.garbage_bits_lc->size() << endl;
          }
        }
        
        int targetLane = lidx + lc_result;
        state[lidx][aidx].lane = targetLane;
        new_state[targetLane].push_back(state[lidx][aidx]);

      }
    }

    for(int lidx = 0; lidx < state.size(); lidx++) {
      sort(new_state[lidx].begin(), new_state[lidx].end());
      reverse(new_state[lidx].begin(), new_state[lidx].end());
    }
    state = new_state;

    for(int lidx = 0; lidx < state.size(); lidx++) {
      for(int aidx = 1; aidx < new_state[lidx].size(); aidx++) {
        if(new_state[lidx][aidx].id && new_state[lidx][aidx].p >= new_state[lidx][aidx - 1].p - veh_length) {
          if(debug) {
            cerr << "ts " << ts << " after lc: an accident occurred on lane " << lidx << " between agents " << new_state[lidx][aidx].id << " and " << new_state[lidx][aidx - 1].id << ", exiting" << endl;
            cerr << "positions after: " << new_state[lidx][aidx].p << " and " << new_state[lidx][aidx - 1].p << endl;
          }
          if(out_filename.compare("") || print_states) {
            for(int lidx = 0; lidx < state.size(); lidx++) {
              for(int aidx = 0; aidx < state[lidx].size(); aidx++) {
                out_stream << lidx << ", " << state[lidx][aidx].id << ", " << ts << ", " << state[lidx][aidx].v << ", " << state[lidx][aidx].p << endl;
              }
            }
          }

          if(out_filename.compare(""))
            out_file.close();
          return false;
        }
      }
    }

    for(int lidx = 0; lidx < state.size(); lidx++) {
      new_state[lidx].clear();
      for(int aidx = 0; aidx < state[lidx].size(); aidx++) {
        Agent new_a = state[lidx][aidx];
        if(!state[lidx][aidx].id) { // skip dummy vehicles
          new_state[lidx].push_back(new_a);
          continue;
        }


        // get out for current in
        Fw_key fw_key = { state[lidx][aidx].v, state[lidx][aidx - 1].v, state[lidx][aidx - 1].p - state[lidx][aidx].p - veh_length};

        if(fw_key.p_delta < 0) {
          cerr << "ts " << ts << ", agent " << new_a.id << " has negative p_delta of " << fw_key.p_delta << ": " << state[lidx][aidx - 1].p << " - " << state[lidx][aidx].p << " - " << veh_length << endl;
          exit(1);
        }
 
        if(fw_key.p_delta > sensing_range)
          fw_key.p_delta = INT32_MAX;

        Fw_val fw_val = fw_table_get(fw_key.v, fw_key.v_ahead, fw_key.p_delta);

        if(debug)
          cout << "agent " << state[lidx][aidx].id << " using " << fw_key.v << ", " << fw_key.v_ahead << ", " << fw_key.p_delta << " -> " << fw_val.new_v << ", " << fw_val.index << ", p_change: " << p_change[fw_val.new_v] << endl;
        fix16_t v_out = fw_val.new_v;
  
        new_a.v = v_out;
        new_a.p += p_change[v_out];
  
        int index = fw_val.index;
  
        Bw_key bw_key = { fw_val.new_v, fw_key.v_ahead, fw_key.p_delta };
        bw_key.index = 0;

        int num_ins = num_fw_keys[bw_key];
        int num_bits = ceil_log2(num_ins - 1);
  
        
        if(new_a.garbage_bits_cf) {
          if(debug)
            cerr << "ts " << ts << ", adding " << num_bits << " bits for agent " << state[lidx][aidx].id << ", currently there are " << state[lidx][aidx].garbage_bits_cf->size() << endl;
          new_a.garbage_bits_cf->write(num_bits, index);
        }

        
        new_state[lidx].push_back(new_a);
        if(new_state[lidx][aidx].id && new_state[lidx][aidx].p >= new_state[lidx][aidx - 1].p - veh_length) {
          if(out_filename.compare("") || print_states) {
            for(int lidx = 0; lidx < new_state.size(); lidx++) {
              for(int aidx = 0; aidx < new_state[lidx].size(); aidx++) {
                out_stream << lidx << ", " << new_state[lidx][aidx].id << ", " << ts << ", " << new_state[lidx][aidx].v << ", " << new_state[lidx][aidx].p << endl;
              }
            }
          }

          if(debug)
            cerr << "ts " << ts << " after cf: an accident occurred on lane " << lidx << " between agents " << new_state[lidx][aidx].id << " and " << new_state[lidx][aidx - 1].id << ", exiting" << endl;
          if(out_filename.compare(""))
            out_file.close();
          return false;
        }
      }
    }

    state = new_state;
  }


  if(out_filename.compare("") || print_states) {
    for(int lidx = 0; lidx < state.size(); lidx++) {
      for(int aidx = 0; aidx < state[lidx].size(); aidx++) {
        out_stream << lidx << ", " << state[lidx][aidx].id << ", " << ts << ", " << state[lidx][aidx].v << ", " << state[lidx][aidx].p << endl;
      }
    }
  }

  return true;
}

void simulate_backward(vector<vector<Agent>> &state, fix16_t sensing_range, int max_ts)
{
  static const fix16_t veh_length = fix16_from_str("4.5");

  if(print_garbage_bit_statistics) {
  
    uint64_t garbage_bits_total = 0;
    uint64_t garbage_bits_total_cf = 0;
    uint64_t garbage_bits_total_lc = 0;

    int num_mobile_vehicles = 0;

    for(int lidx = 0; lidx < state.size(); lidx++) {
      for(int aidx = 1; aidx < state[lidx].size(); aidx++) {
        Agent &a = state[lidx][aidx];
        if(!a.id)
          continue;

        num_mobile_vehicles++;
  
        const int garbage_bits_vehicle = a.garbage_bits_cf->size() + a.garbage_bits_lc->size();
  
        cerr << "vehicle " << a.id << ", garbage bits: " << garbage_bits_vehicle << endl;
        garbage_bits_total += garbage_bits_vehicle; 
        garbage_bits_total_cf += a.garbage_bits_cf->size();
        garbage_bits_total_lc += a.garbage_bits_lc->size();
      }
    }
    cerr << "total garbage bits: " << garbage_bits_total << ", " << ceil_div(garbage_bits_total, 8) << " bytes, " << (float)garbage_bits_total / (8 * 1024) << " KiB" << endl;
  
    const float bits_per_time_step_per_vehicle = (float)garbage_bits_total / max_ts / num_mobile_vehicles;
  
    cerr << "total per time step: " << (float)garbage_bits_total / max_ts << " bits, " << (float)ceil_div(garbage_bits_total, 8) / max_ts << " bytes" << endl;
    cerr << "per time step per vehicle: " << bits_per_time_step_per_vehicle << " bits, " << (float)ceil_div(garbage_bits_total, 8) / max_ts / num_mobile_vehicles << " bytes" << endl;
    cerr << "per time step per vehicle, cf only: " << (float)garbage_bits_total_cf / max_ts / num_mobile_vehicles << " bits, " << (float)ceil_div(garbage_bits_total_cf, 8) / max_ts / num_mobile_vehicles << " bytes" << endl;
    cerr << "per time step per vehicle, lc only: " << (float)garbage_bits_total_lc / max_ts / num_mobile_vehicles << " bits, " << (float)ceil_div(garbage_bits_total_lc, 8) / max_ts / num_mobile_vehicles << " bytes" << endl;
  }

  for(int ts = max_ts - 1; ts >= 0; ts--) {
    for(int lidx = 0; lidx < state.size(); lidx++) {
      for(int aidx = 1; aidx < state[lidx].size(); aidx++) {
        Agent &a = state[lidx][aidx];

        if(!a.id)
          continue;

        fix16_t old_p = a.p - p_change[a.v];
        fix16_t old_p_delta = state[lidx][aidx - 1].p - old_p - veh_length;
        if(old_p_delta > sensing_range)
          old_p_delta = INT32_MAX;

        // get num garbage bits for out
        // read bits from bitvector, set index in out
        Bw_key bw_key = { a.v, state[lidx][aidx - 1].v, old_p_delta, 0 };

        // get num garbage bits for out
        int num_ins = num_fw_keys[bw_key];
        if(num_ins == 0) {
          cerr << "ts " << ts << ", vehicle " << a.id << ", this situation is unreachable: bw key " << bw_key.new_v << ", " << bw_key.old_v_ahead << ", " << bw_key.old_p_delta << ", " << bw_key.index << endl;
          exit(1);
        }

        if(debug)
          cerr << "num_ins is " << num_ins << endl;
        int num_bits = ceil_log2(num_ins - 1);

        if(debug)
          cerr << "cf trying to read " << num_bits << " bits for agent " << a.id << ", " << bw_key.new_v << ", " << bw_key.old_v_ahead << ", " << bw_key.old_p_delta << ", " << bw_key.index << endl;
        if(!a.garbage_bits_cf) {
          cout << "BUG! agent does not have garbage bits" << endl;
          exit(1);
        }          

        if(debug) {
          cerr << "ts " << ts << ", removing " << num_bits << " bits for agent " << a.id << ", currently there are " << a.garbage_bits_cf->size() << endl;
          cerr << "agent " << a.id << " using " << bw_key.new_v << ", " << bw_key.old_v_ahead << ", " << bw_key.old_p_delta << ", " << bw_key.index << endl;
        }

        bw_key.index = a.garbage_bits_cf->read(num_bits, true);

        // we have: new_v, old_v_ahead, old_p_delta; we want: old_v
        auto it = bw_table.find(bw_key);
        if(it == bw_table.end()) {
          cerr << "agent " << a.id << ": no v for bw key " << bw_key.new_v << ", " << bw_key.old_v_ahead << ", " << bw_key.old_p_delta << ", " << bw_key.index << endl;

          exit(1);
        }
 
        a.v = it->second;
        a.p = old_p;
      }
    }

    vector<vector<Agent>> prev_state(state.size());

    for(int lidx = 0; lidx < state.size(); lidx++) {
      prev_state[lidx].push_back(state[lidx][0]);
      for(int aidx = 1; aidx < state[lidx].size(); aidx++) {
        Agent &a = state[lidx][aidx];

        int lc_result = 0;

        if(a.id) {
          if(!a.garbage_bits_lc) {
            cerr << "BUG! agent does not have garbage bits" << endl;
            exit(1);
          }

          if(debug) {
            cerr << "lc trying to read " << 2 << " bits for agent " << state[lidx][aidx].id << ", has: " << state[lidx][aidx].garbage_bits_lc->size() << endl;
            cerr << ts << ", " <<  ts % epoch_length << ", lc check for agent " << a.id << ", garbage bits left: " << a.garbage_bits_lc->size() << endl;
          }

          int marker = a.garbage_bits_lc->read(1, false);

          if(debug)
            cerr << "lc check for agent " << a.id << ", marker is " << marker << endl;

          if(marker) {
            int bits = a.garbage_bits_lc->read(epoch_bits + 1, false);
            int lc_ts = bits >> 1;

            if(debug)
              cerr << "time step is " << lc_ts << endl;

            if(lc_ts == ts % epoch_length) {
              bits = a.garbage_bits_lc->read(epoch_bits + 2, true);
              int lc_bit_ = bits >> (epoch_bits + 1);
              if(debug)
                cerr << "lc_bit is " << lc_bit_ << endl;

              bool lc_bit = bits >> (epoch_bits + 1);
              lc_result = lc_bit ? 1 : -1;

              if(debug)
                cerr << "lc for vehicle " << a.id << " at ts " << ts << ", lc_result: " << lc_result << ", moving from " << lidx << " to " << lidx - lc_result << ", garbage bits left: " << a.garbage_bits_lc->size() << endl;
            }
          }

          if(ts && !(ts % epoch_length)) {
            int epoch_marker = a.garbage_bits_lc->read(1, true);
            if(epoch_marker) {
              cerr << "BUG! expected an epoch marker for agent " << a.id << endl;
              exit(1);
            }
          }

        }
        int target_lane = lidx - lc_result;
        prev_state[target_lane].push_back(a);
        prev_state[target_lane].rbegin()->lane = target_lane;
      }
    }

    for(int lidx = 0; lidx < prev_state.size(); lidx++) {
      sort(prev_state[lidx].begin(), prev_state[lidx].end());
      reverse(prev_state[lidx].begin(), prev_state[lidx].end());
    }

    state = prev_state;

    if(print_states) {
      for(int lidx = 0; lidx < state.size(); lidx++) {
        for(int aidx = 0; aidx < state[lidx].size(); aidx++) {
          cout << lidx << ", " << state[lidx][aidx].id << ", " << ts << ", " << state[lidx][aidx].v << ", " << state[lidx][aidx].p << endl;
        }
      }
    }
  }
}

bool run_reversible_sim_evaluation(fix16_t sensing_range, fix16_t p_step, fix16_t v_step, int num_mobile_vehicles, int num_obstacles, int max_ts)
{
  static const fix16_t veh_length = fix16_from_str("4.5");
  vector<vector<Agent>> state(num_lanes);

  int num_vehicles = num_mobile_vehicles + num_lanes + num_obstacles;

  Agent a;
  for(int lidx = 0; lidx < num_lanes; lidx++) {
    state[lidx].clear();
    a.id = 0;
    a.v = fix16_from_int(0);
    a.p = fix16_from_int(20000);
    a.lane = lidx;
    state[lidx].push_back(a);
  }

  
  a.p = 0;
  for(int i = 0; i < num_obstacles; i++) {
    a.id = 0;
    a.v = 0;
    uint32_t factor_p = rand() % (fix16_from_int(40) / p_step + 1) + fix16_from_int(10) / p_step;

    a.p += p_step * factor_p;
    a.lane = rand() % num_lanes;

    state[a.lane].push_back(a);
  }

  int next_id = 1;
  for(int i = 0; i < num_mobile_vehicles; i++) {
    a.id = next_id++;

    uint32_t factor_v = rand() % (fix16_from_int(20) / v_step + 1);
    a.v = v_step * factor_v;

    uint32_t factor_p = rand() % (fix16_from_int(500) / p_step + 1);
    a.p = p_step * factor_p;

    a.lane = rand() % num_lanes;

    a.init_garbage_bits();

    state[a.lane].push_back(a);
  }

  for(int lidx = 0; lidx < num_lanes; lidx++) {
    sort(state[lidx].begin(), state[lidx].end());
    reverse(state[lidx].begin(), state[lidx].end());
  }

  for(int lidx = 0; lidx < state.size(); lidx++) {
    for(int aidx = 1; aidx < state[lidx].size(); aidx++) {
      if(state[lidx][aidx].p >= state[lidx][aidx - 1].p - veh_length) {
        if(debug)
          cerr << "generated multiple agents in the same spot: " << state[lidx][aidx].p << " and " << state[lidx][aidx].p << endl;
        for(int lidx = 0; lidx < state.size(); lidx++) {
          for(int aidx = 0; aidx < state[lidx].size(); aidx++) {
            Agent &a = state[lidx][aidx];
            if(a.id)
              a.delete_garbage_bits();
          }
        }

        return false;
      }
    }
  }

  pt::ptime timeStart = pt::microsec_clock::local_time();
  int final_ts = max_ts;
  bool r = simulate_forward(state, sensing_range, max_ts, final_ts);
  if(!r) {
    for(int lidx = 0; lidx < state.size(); lidx++) {
      for(int aidx = 0; aidx < state[lidx].size(); aidx++) {
        Agent &a = state[lidx][aidx];
        if(a.id)
          a.delete_garbage_bits();
      }
    }

    return false;
  }

  pt::ptime timeFinish = pt::microsec_clock::local_time();

  cerr << "forward simulation took " << (timeFinish - timeStart).total_microseconds() << " us" << endl;

  timeStart = pt::microsec_clock::local_time();
  simulate_backward(state, sensing_range, max_ts);
  timeFinish = pt::microsec_clock::local_time();

  cerr << "backward simulation took " << (timeFinish - timeStart).total_microseconds() << " us" << endl;

  for(int lidx = 0; lidx < state.size(); lidx++) {
    for(int aidx = 0; aidx < state[lidx].size(); aidx++) {
      Agent &a = state[lidx][aidx];
      if(a.id)
        a.delete_garbage_bits();
    }
  }

  return true;
}

bool run_case_study(fix16_t sensing_range, fix16_t p_step, fix16_t v_step, int num_mobile_vehicles, int num_obstacles, int max_ts)
{
  static const fix16_t veh_length = fix16_from_str("4.5");
  vector<vector<Agent>> state(num_lanes);

  int num_vehicles = num_mobile_vehicles + num_lanes + num_obstacles;

  int factor_v[2];

  int max_overlap_factor = veh_length / p_step + 1;
  int max_obst_dist_factor = fix16_from_int(50) / p_step + 1;
  int max_factor_v = fix16_from_int(20) / v_step + 1;

  int rough_total = max_obst_dist_factor * max_obst_dist_factor * max_factor_v;

  int old_rev_explore_solutions = 0;

  int iter = 0;

  for(int overlap_factor = max_overlap_factor - 1; overlap_factor >= 0; overlap_factor--) {
    for(int obst_dist_factor = 0; obst_dist_factor < max_obst_dist_factor; obst_dist_factor++) {
      for(factor_v[0] = 0; factor_v[0] < max_factor_v; factor_v[0]++) {
        for(factor_v[1] = 0; factor_v[1] < max_factor_v; factor_v[1]++) {
          iter++;

          if(iter % 100000 == 0)
            cerr << "roughly " << 100 * iter / rough_total << " percent done, " << iter << " out of " << rough_total << endl;

          Agent a;
          for(int lidx = 0; lidx < num_lanes; lidx++) {
            state[lidx].clear();
            a.id = 0;
            a.v = fix16_from_int(0);
            a.p = fix16_from_int(10000);
            a.lane = lidx;
            state[lidx].push_back(a);
          }

          a.id = 0;
          a.v = 0;
          a.p = fix16_from_int(150);
          a.lane = 0;

          state[a.lane].push_back(a);

          a.id = 0;
          a.v = 0;
          a.p = fix16_from_int(150);
          a.lane = 2;

          state[a.lane].push_back(a);

          int next_id = 1;
          for(int i = 0; i < num_mobile_vehicles; i++) {
            a.id = next_id++;

            a.v = v_step * factor_v[i];
            if(i == 0)
              a.p = fix16_from_int(150) - veh_length - obst_dist_factor * p_step;
            else
              a.p = fix16_from_int(150) - veh_length - obst_dist_factor * p_step - overlap_factor * p_step;

            a.init_garbage_bits();

            a.lane = 1;

            state[a.lane].push_back(a);
          }

          for(int lidx = 0; lidx < num_lanes; lidx++) {
            sort(state[lidx].begin(), state[lidx].end());
            reverse(state[lidx].begin(), state[lidx].end());
          }

          int final_ts = max_ts;

          reverse_explore(state, final_ts, 0, sensing_range, num_lanes, num_obstacles, final_ts, true, true);

          if(rev_explore_solutions != old_rev_explore_solutions) {
            old_rev_explore_solutions = rev_explore_solutions;
            dump_state(state, "final state");
          }

          for(int lidx = 0; lidx < state.size(); lidx++) {
            for(int aidx = 0; aidx < state[lidx].size(); aidx++) {
              Agent &a = state[lidx][aidx];
              if(a.id)
                a.delete_garbage_bits();
            }
          }

        }
      }
    }
  }
  return true;
}

bool run_rev_exploration_evaluation(fix16_t sensing_range, fix16_t p_step, fix16_t v_step, int num_mobile_vehicles, int num_obstacles, int max_ts)
{
  static const fix16_t veh_length = fix16_from_str("4.5");
  vector<vector<Agent>> state(num_lanes);

  int num_vehicles = num_mobile_vehicles + num_lanes + num_obstacles;

  Agent a;
  for(int lidx = 0; lidx < num_lanes; lidx++) {
    state[lidx].clear();
    a.id = 0;
    a.v = fix16_from_int(0);
    a.p = fix16_from_int(20000);
    a.lane = lidx;
    state[lidx].push_back(a);
  }

  a.p = 0;
  for(int i = 0; i < num_obstacles; i++) {
    a.id = 0;
    a.v = 0;
    uint32_t factor_p = rand() % (fix16_from_int(40) / p_step + 1) + fix16_from_int(10) / p_step;

    a.p += p_step * factor_p;
    a.lane = rand() % num_lanes;

    state[a.lane].push_back(a);
  }

  int next_id = 1;
  for(int i = 0; i < num_mobile_vehicles; i++) {
    a.id = next_id++;

    uint32_t factor_v = rand() % (fix16_from_int(20) / v_step + 1);
    cerr << factor_v << endl;
    a.v = v_step * factor_v;

    uint32_t factor_p = rand() % (fix16_from_int(100) / p_step + 1);
    a.p = p_step * factor_p;

    a.lane = rand() % num_lanes;

    a.init_garbage_bits();

    state[a.lane].push_back(a);
  }

  for(int lidx = 0; lidx < num_lanes; lidx++) {
    sort(state[lidx].begin(), state[lidx].end());
    reverse(state[lidx].begin(), state[lidx].end());
  }

  for(int lidx = 0; lidx < state.size(); lidx++) {
    for(int aidx = 1; aidx < state[lidx].size(); aidx++) {
      if(state[lidx][aidx].p >= state[lidx][aidx - 1].p - veh_length) {
        if(debug)
          cerr << "generated multiple agents in the same spot: " << state[lidx][aidx].p << " and " << state[lidx][aidx].p << endl;
        for(int lidx = 0; lidx < state.size(); lidx++) {
          for(int aidx = 0; aidx < state[lidx].size(); aidx++) {
            Agent &a = state[lidx][aidx];
            if(a.id)
              a.delete_garbage_bits();
          }
        }

        return false;
      }
    }
  }

  int final_ts = max_ts;
  bool r = simulate_forward(state, sensing_range, max_ts, final_ts);
  if(!r) {
    for(int lidx = 0; lidx < state.size(); lidx++) {
      for(int aidx = 0; aidx < state[lidx].size(); aidx++) {
        Agent &a = state[lidx][aidx];
        if(a.id)
          a.delete_garbage_bits();
      }
    }

    return false;
  }

  reverse_explore(state, final_ts, 0, sensing_range, num_lanes, num_obstacles, final_ts, false, false);

  for(int lidx = 0; lidx < state.size(); lidx++) {
    for(int aidx = 0; aidx < state[lidx].size(); aidx++) {
      Agent &a = state[lidx][aidx];
      if(a.id)
        a.delete_garbage_bits();
    }
  }

  return true;
}

int main(int argc, char **argv)
{
  int mode = atoi(argv[1]);
  int num_runs = atoi(argv[2]);
  int seed = atoi(argv[3]);

  fix16_t sensing_range = atoi(argv[4]);

  fix16_t v_step = atoi(argv[5]);
  fix16_t p_step = atoi(argv[6]);

  double dt_dbl = atof(argv[7]);

  int num_mobile_vehicles = atoi(argv[8]);
  int num_obstacles = atoi(argv[9]);
  int max_ts = atoi(argv[10]);

  create_tables(sensing_range, v_step, p_step, dt_dbl, mode == 2);

  if(mode == 2)
    exit(1);

  int successes = 0;

  do {
    srand(seed);

    if(debug)
      cerr << "seed is " << seed << endl;
    
    bool r;

    if(!mode)
      r = run_reversible_sim_evaluation(sensing_range, v_step, p_step, num_mobile_vehicles, num_obstacles, max_ts);
    else if(mode == 1)
      r = run_rev_exploration_evaluation(sensing_range, v_step, p_step, num_mobile_vehicles, num_obstacles, max_ts);
    else if(mode == 3)
      r = run_case_study(sensing_range, v_step, p_step, num_mobile_vehicles, num_obstacles, max_ts);

    if(r)
      successes++;

    seed++;

  } while(successes < num_runs);

  if(debug)
    cerr << "returning " << seed - 1 << endl;

  return seed - 1;
}
