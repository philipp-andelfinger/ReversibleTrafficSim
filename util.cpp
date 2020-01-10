#include "util.hpp"

int ceil_log2(int x)
{
  int r = 0;

  for(int remainder = x; remainder > 0; remainder /= 2, r++);

  return r;
}

void split_state_by_lane(vector<Agent> &state, vector<vector<Agent>> &split_state, int num_lanes)
{
  for(auto it = state.begin(); it != state.end(); it++) {
    int lane = it->lane;

    if(lane < 0 || lane > num_lanes - 1) {
      cerr << "BUG! agent " << it->id << " claims to be on lane " << it->lane << endl;
      exit(1);
    }

    split_state[lane].push_back(*it);
  }
}

void flatten_state(vector<vector<Agent>> &state, vector<Agent> &linear_state)
{
  for(auto it = state.begin(); it != state.end(); it++) {
    for(auto iit = it->begin(); iit != it->end(); iit++) {
      linear_state.push_back(*iit);
    }
  }
}

void dump_state(vector<vector<Agent>> &state, const char *str)
{
  cerr << endl << str << endl;
  for(int lidx = 0; lidx < state.size(); lidx++) {
    for(int aidx = 0; aidx < state[lidx].size(); aidx++) {
      //if(!state[lidx][aidx].id)
      //  continue;
      cerr << state[lidx][aidx].lane << ", " << state[lidx][aidx].id << ", " << state[lidx][aidx].v << ", " << state[lidx][aidx].p << endl;
    }
  }
}

void dump_state(vector<Agent> &state, const char *str)
{
  cerr << endl << str << endl;
  for(int aidx = 0; aidx < state.size(); aidx++) {
    //if(!state[aidx].id)
    //  continue;

    cerr << state[aidx].lane << ", " << state[aidx].id << ", " << state[aidx].v << ", " << state[aidx].p << endl;
  }
}

int ceil_div(int x, int y)
{
  return (x + y - 1) / y;
}

int ipow(int x, int y)
{
  int r = 1;

  while(y-- > 0)
    r *= x;

  return r;
}

double ipow(double x, int y)
{
  double r = 1;

  while(y-- > 0)
    r *= x;

  return r;
}
