#pragma once

#include <boost/functional/hash.hpp>

#include <algorithm>
#include <utility>

#include <set>
#include <vector>
#include <map>
#include <unordered_map>
#include <string>

#include <fixmath/fixmath.h>
#include <boost/multiprecision/cpp_int.hpp>

#include "bitvector.hpp"

using namespace std;

class Agent {
public:
  int id;
  int lane;
  fix16_t v, p;

  BitVector *garbage_bits_cf;
  BitVector *garbage_bits_lc;

  void init_garbage_bits()
  {
    garbage_bits_cf = new BitVector();
    garbage_bits_lc = new BitVector();
  }

  void delete_garbage_bits()
  {
    delete garbage_bits_cf;
    delete garbage_bits_lc;
  }

  bool operator<(const Agent &other) const { return (lane > other.lane || (lane == other.lane && p < other.p) || (lane == other.lane && p == other.p && v < other.v) || (lane == other.lane && p == other.p && v == other.v && id < other.id)); };
  bool operator!=(const Agent &other) const { return id != other.id || lane != other.lane || v != other.v || p != other.p; };
};

void split_state_by_lane(vector<Agent> &state, vector<vector<Agent>> &split_state, int num_lanes);
void flatten_state(vector<vector<Agent>> &state, vector<Agent> &linear_state);
void dump_state(vector<vector<Agent>> &state, const char *str);
void dump_state(vector<Agent> &state, const char *str);
int ceil_div(int x, int y);
int ceil_log2(int x);
int ipow(int x, int y);
double ipow(double x, int y);

struct Fw_key {
  fix16_t v; 
  fix16_t v_ahead;
  fix16_t p_delta;

  bool operator==(const Fw_key &other) const { return (v == other.v && p_delta == other.p_delta && v_ahead == other.v_ahead); }
  bool operator!=(const Fw_key &other) const { return (v != other.v || p_delta != other.p_delta || v_ahead != other.v_ahead); }
  bool operator<(const Fw_key &other) const { return (v < other.v || (v == other.v && p_delta < other.p_delta) || (v == other.v && p_delta == other.p_delta && v_ahead < other.v_ahead)); }
};

namespace std {
  template <>
  struct hash<Fw_key> {
    size_t operator()(const Fw_key& key) const
    {
      size_t result = 0;
      boost::hash_combine(result, key.v);
      boost::hash_combine(result, key.v_ahead); 
      boost::hash_combine(result, key.p_delta); 
      return result;
    }
  };
}

struct Bw_key {
  fix16_t new_v;
  fix16_t old_v_ahead;
  fix16_t old_p_delta;
  uint32_t index;

  bool operator==(const Bw_key &other) const { return (new_v == other.new_v && old_v_ahead == other.old_v_ahead && old_p_delta == other.old_p_delta && index == other.index); }
};

namespace std {
  template <>
  struct hash<Bw_key>
  {
    size_t operator()(const Bw_key& key) const
    {
      size_t result = 0;
      boost::hash_combine(result, key.new_v);
      boost::hash_combine(result, key.old_v_ahead); 
      boost::hash_combine(result, key.old_p_delta); 
      boost::hash_combine(result, key.index); 
      return result;
    }
  };
}

struct Fw_val {
  fix16_t new_v;
  uint32_t index;

  bool operator==(const Bw_key &other) const { return (new_v == other.new_v && index == other.index); }
};

namespace std {
  template <>
  struct hash<Fw_val>
  {
    size_t operator()(const Fw_val& val) const
    {
      size_t result = 0;
      boost::hash_combine(result, val.new_v);
      boost::hash_combine(result, val.index); 
      return result;
    }
  };
}
