#include <unistd.h>
#include "tables.hpp"
#include "cf.hpp"

Fw_val ***fw_table;

unordered_map<Bw_key, fix16_t> bw_table;
unordered_map<Bw_key, uint32_t> num_fw_keys;
unordered_map<fix16_t, fix16_t> p_change;

fix16_t m_v_step;
fix16_t m_p_step;

uint64_t m_dim_x;
uint64_t m_dim_y;
uint64_t m_dim_z;

static const bool debug = true;

Fw_val fw_table_access(fix16_t v, fix16_t v_ahead, fix16_t p_delta, bool write, Fw_val fw_val)
{
  uint64_t v_inc = v / m_v_step;
  uint64_t v_ahead_inc = v_ahead / m_v_step;
  uint64_t p_delta_inc = p_delta / m_p_step;

  if(p_delta == INT32_MAX)
    p_delta_inc = m_dim_z - 1;

  if(write) {
    fw_table[v_inc][v_ahead_inc][p_delta_inc] = fw_val;
    return Fw_val();
  }

  return fw_table[v_inc][v_ahead_inc][p_delta_inc];
}

Fw_val fw_table_get(fix16_t v, fix16_t v_ahead, fix16_t p_delta)
{
  return fw_table_access(v, v_ahead, p_delta, false, Fw_val());
}

void fw_table_set(fix16_t v, fix16_t v_ahead, fix16_t p_delta, Fw_val fw_val)
{
  fw_table_access(v, v_ahead, p_delta, true, fw_val);
}

bool read_tables(fix16_t sensing_range, fix16_t v_step, fix16_t p_step, double dt_dbl, unordered_map<Bw_key, vector<Fw_key>> &bw_table_ambig)
{
  const int max_str_len = 128;
  char table_filename[max_str_len];
  snprintf(table_filename, max_str_len, "%d_%d_%d_%.2f.dat", sensing_range, v_step, p_step, dt_dbl);

  if(access(table_filename, F_OK) == -1)
    return false;

  if(debug)
    cerr << "reading tables from file" << endl;

  FILE *stream = fopen(table_filename, "r");

  size_t num_fw_bw_table_keys, num_p_change_keys, num_bw_table_ambig_keys;
  
  fread(&num_fw_bw_table_keys, sizeof(size_t), 1, stream);
  fread(&num_bw_table_ambig_keys, sizeof(size_t), 1, stream);
  fread(&num_p_change_keys, sizeof(size_t), 1, stream);
 
  if(debug) {
    cerr << "table sizes: " << num_fw_bw_table_keys << ", " << num_bw_table_ambig_keys << ", " << num_p_change_keys << endl;
    cerr << "memory consumption: fw: " << ((sizeof(Fw_key) + sizeof(Bw_key)) * num_fw_bw_table_keys) / 1024 / 1024 << "MiB, bw: " << ((sizeof(Bw_key) + sizeof(fix16_t)) * num_bw_table_ambig_keys) / 1024 / 1024 << "MiB, num_ins: " << ((sizeof(Bw_key) + sizeof(uint32_t)) * num_bw_table_ambig_keys) / 1024 / 1024 << "MiB" << endl;
  }

  for(int i = 0; i < num_fw_bw_table_keys; i++) {
    Fw_key fw_key;
    Fw_val fw_val;
     
    fread(&fw_key, sizeof(Fw_key), 1, stream);
    fread(&fw_val, sizeof(Fw_val), 1, stream);

    fw_table_set(fw_key.v, fw_key.v_ahead, fw_key.p_delta, fw_val);
  }

  for(int i = 0; i < num_fw_bw_table_keys; i++) {
    Bw_key bw_key;
    fix16_t v;
     
    fread(&bw_key, sizeof(Bw_key), 1, stream);
    fread(&v, sizeof(fix16_t), 1, stream);

    bw_table[bw_key] = v;
  }

  for(int i = 0; i < num_bw_table_ambig_keys; i++) {
    Bw_key bw_key;
    uint32_t num_fw_keys_;

    fread(&bw_key, sizeof(Bw_key), 1, stream);
    fread(&num_fw_keys_, sizeof(uint32_t), 1, stream);

    num_fw_keys[bw_key] = num_fw_keys_;
  }

  for(int i = 0; i < num_p_change_keys; i++) {
    fix16_t v;
    fix16_t p_change_;

    fread(&v, sizeof(fix16_t), 1, stream);
    fread(&p_change_, sizeof(fix16_t), 1, stream);

    p_change[v] = p_change_;
  }

  fclose(stream);

  return true;
}

void write_tables(fix16_t sensing_range, fix16_t v_step, fix16_t p_step, double dt_dbl, unordered_map<Bw_key, vector<Fw_key>> &bw_table_ambig)
{
  const int max_str_len = 128;
  char table_filename[max_str_len];
  snprintf(table_filename, max_str_len, "%d_%d_%d_%.2f.dat", sensing_range, v_step, p_step, dt_dbl);

  FILE *stream = fopen(table_filename, "w");

  size_t buf[] = { m_dim_x * m_dim_y * m_dim_z, bw_table_ambig.size(), p_change.size() };
  fwrite(&buf, sizeof(size_t), sizeof(buf) / sizeof(size_t), stream);
  
  for(fix16_t v_in = fix16_from_int(0); v_in <= fix16_from_int(20); v_in += v_step) {
    for(fix16_t v_ahead_in = fix16_from_int(0); v_ahead_in <= fix16_from_int(20); v_ahead_in += v_step) {
      for(fix16_t p_delta_in_ = fix16_from_int(0); p_delta_in_ <= sensing_range + p_step; p_delta_in_ += p_step) {
        fix16_t p_delta_in;
        if(p_delta_in_ > sensing_range) {
          p_delta_in = INT32_MAX;
        } else {
          p_delta_in = p_delta_in_;
        }
 
        Fw_key fw_key = {v_in, v_ahead_in, p_delta_in_};
        Fw_val fw_val = fw_table_get(v_in, v_ahead_in, p_delta_in_);
         
        fwrite(&fw_key, sizeof(Fw_key), 1, stream);
        fwrite(&fw_val, sizeof(Fw_val), 1, stream);
      }
    }
  }

  for(auto it = bw_table.begin(); it != bw_table.end(); it++) {
    Bw_key bw_key = it->first;
    fix16_t v = it->second;
     
    fwrite(&bw_key, sizeof(Bw_key), 1, stream);
    fwrite(&v, sizeof(fix16_t), 1, stream);
  }

  for(auto it = bw_table_ambig.begin(); it != bw_table_ambig.end(); it++) {
    Bw_key bw_key = it->first;
    uint32_t num_fw_keys_ = num_fw_keys[bw_key];

    fwrite(&bw_key, sizeof(Bw_key), 1, stream);
    fwrite(&num_fw_keys_, sizeof(uint32_t), 1, stream);
  }

  for(auto it = p_change.begin(); it != p_change.end(); it++) {
    fix16_t v = it->first;
    fix16_t p_change_ = it->second;

    fwrite(&v, sizeof(fix16_t), 1, stream);
    fwrite(&p_change_, sizeof(fix16_t), 1, stream);
  }

  fclose(stream);
}

void create_tables(fix16_t sensing_range, fix16_t v_step, fix16_t p_step, double dt_dbl, bool dump_input_transitions)
{
  m_p_step = p_step;
  m_v_step = v_step;

  m_dim_x = fix16_from_int(20) / v_step + 1;
  m_dim_y = fix16_from_int(20) / v_step + 1;
  m_dim_z = sensing_range / p_step + 2;

  fw_table = new Fw_val**[m_dim_x];
  for(int x = 0; x < m_dim_x; x++) {
    fw_table[x] = new Fw_val*[m_dim_y];
    for(int y = 0; y < m_dim_y; y++) {
      fw_table[x][y] = new Fw_val[m_dim_z];
    }
  }

  unordered_map<Bw_key, vector<Fw_key>> bw_table_ambig;

  map<fix16_t, uint64_t> num_fw_keys_same_new_v;

  if(!dump_input_transitions && read_tables(sensing_range, v_step, p_step, dt_dbl, bw_table_ambig)) {
    return;
  }

  const uint64_t num_combinations = m_dim_x * m_dim_y * m_dim_z;
  int curr_combination = 0;

  cerr << "creating initial tables" << endl;
  cerr << "num_combinations: " << num_combinations << endl;
  cerr << "disregarding overhead, the size of the tables is " << (num_combinations * (20 + 20 + 16)) << " bytes" << endl;

  for(fix16_t v_in = fix16_from_int(0); v_in <= fix16_from_int(20); v_in = v_in += v_step) {
    for(fix16_t v_ahead_in = fix16_from_int(0); v_ahead_in <= fix16_from_int(20); v_ahead_in += v_step) {
      for(fix16_t p_delta_in_ = fix16_from_int(0); p_delta_in_ <= sensing_range + p_step; p_delta_in_ += p_step) {
        double v_ahead_in_dbl = fix16_to_dbl(v_ahead_in);

        fix16_t p_delta_in;
        double p_delta_in_dbl;
        if(p_delta_in_ > sensing_range) {
          p_delta_in_dbl = INFINITY;
          p_delta_in = INT32_MAX;
        } else {
          p_delta_in_dbl = fix16_to_dbl(p_delta_in_);
          p_delta_in = p_delta_in_;
        }
  
        double v_in_dbl = fix16_to_dbl(v_in);
  
        double a_out_dbl = idm(v_in_dbl, v_ahead_in_dbl - v_in_dbl, p_delta_in_dbl);
        double v_out_dbl = v_in_dbl + a_out_dbl * dt_dbl;

        if(v_out_dbl < 0.0)
          v_out_dbl = 0.0;

        double p_change_out_dbl = v_out_dbl * dt_dbl;
  
        // FIXME: in the paper, we include negative velocities in the table.
        //        the final version will exclude them.
        fix16_t v_out = fix16_from_dbl(v_out_dbl);
        // fix16_t v_out = max(fix16_from_dbl(v_out_dbl), 0);

        if(v_out % v_step != 0)
          v_out = v_out + v_step - v_out % v_step;

        // FIXME: see above
        //if(v_out < 0) {
        //  cerr << "BUG! v_out < 0" << endl;
        //  exit(1);
        //}

        fix16_t p_change_out = fix16_from_dbl(p_change_out_dbl);

        if(p_change_out % p_step != 0)
          p_change_out = p_change_out + p_step - p_change_out % p_step;

        p_change[v_out] = p_change_out;
  
        Fw_key fw_key = { v_in, v_ahead_in, p_delta_in };

        Bw_key bw_key = { v_out, v_ahead_in, p_delta_in, 0 };

        bw_table_ambig[bw_key].push_back(fw_key);

        curr_combination++;
        if(curr_combination % 1000000 == 0) {
          cerr << 100.0 * curr_combination / num_combinations << "%\r";
        }
      }
    }
  }

  if(debug)
    cerr << "creating final tables" << endl;

  if(dump_input_transitions)
    cerr << "unique backward keys: " << bw_table_ambig.size() << endl;

  for(auto it = bw_table_ambig.begin(); it != bw_table_ambig.end(); it++) {
    Bw_key bw_key = it->first;

    auto in_set = it->second;
    num_fw_keys[bw_key] = in_set.size();

    if(dump_input_transitions) {
      num_fw_keys_same_new_v[bw_key.new_v] += in_set.size();
    }

    uint32_t index = 0;
    for(auto sit = in_set.begin(); sit != in_set.end(); sit++) {
      Fw_key fw_key = *sit;
      bw_key.index = index;
      bw_table[bw_key] = fw_key.v;

      Fw_val fw_val = { bw_key.new_v, bw_key.index };

      fw_table_set(fw_key.v, fw_key.v_ahead, fw_key.p_delta, fw_val);

      index++;
    }

    vector<Fw_key>().swap(it->second);
  }

  for(auto it = num_fw_keys_same_new_v.begin(); it != num_fw_keys_same_new_v.end(); it++) {
    cerr << "input transitions: " << it->first << ", " << it->second << endl;
  }

  write_tables(sensing_range, v_step, p_step, dt_dbl, bw_table_ambig);
}
