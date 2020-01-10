#include <math.h>
#include <vector>
#include "lc.hpp"
#include "cf.hpp"

using namespace std;

void findClosestVehicles(const vector<Agent> &lane, int lane_idx, fix16_t own_p, int own_idx, fix16_t own_lane, int veh_indexes[2])
{
  veh_indexes[0] = veh_indexes[1] = -1;

  for(int i = 0; i < lane.size(); i++) {
    if(i == own_idx)
      continue;
    if(lane[i].id && lane[i].p < own_p || (lane[i].p == own_p && (own_lane != lane_idx || own_idx != i))) {
      veh_indexes[1] = i;
      return;
    } else if(lane[i].p >= own_p) {
      veh_indexes[0] = i;
    }
  }
}

int lc(const vector<vector<Agent>> &state, int veh_lane, int veh_index, fix16_t v_max)
{
  const float politeness = 0.1;
  const float safe_decel = -3.0;
  const float incThreshold = 1.0;
  const float veh_length = 4.5;

  fix16_t position = state[veh_lane][veh_index].p;
  fix16_t velocity = state[veh_lane][veh_index].v;

  int veh_indexes[6] = { -1, -1, -1, -1, -1, -1 };

  const float v_max_float = fix16_to_float(v_max);

  float vCurrent = fix16_to_float(velocity);
  float vFrontVehLeft = v_max_float;
  float vFrontVehCurrent = v_max_float;
  float vFrontVehRight = v_max_float;
  float gapFrontLeft = INFINITY;
  float gapFrontCurrent = INFINITY;
  float gapFrontRight = INFINITY;
  float vRearVehLeft = 0.0;
  float vRearVehCurrent = 0.0;
  float vRearVehRight = 0.0;
  float gapRearLeft = INFINITY;
  float gapRearCurrent = INFINITY;
  float gapRearRight = INFINITY;

  if(veh_lane > 0)
    findClosestVehicles(state[veh_lane - 1], veh_lane - 1, position, veh_index, veh_lane, &veh_indexes[0]);
  findClosestVehicles(state[veh_lane], veh_lane, position, veh_index, veh_lane, &veh_indexes[2]);
  if(veh_lane < state.size() - 1)
    findClosestVehicles(state[veh_lane + 1], veh_lane, position, veh_index, veh_lane, &veh_indexes[4]);

  if(veh_indexes[0] != -1) {
    vFrontVehLeft = fix16_to_float(state[veh_lane - 1][veh_indexes[0]].v);
    gapFrontLeft = fix16_to_float(state[veh_lane - 1][veh_indexes[0]].p - position) - veh_length;
  }

  if(veh_indexes[1] != -1) {
    vRearVehLeft = fix16_to_float(state[veh_lane - 1][veh_indexes[1]].v);
    gapRearLeft = fix16_to_float(position - state[veh_lane - 1][veh_indexes[1]].p) - veh_length;
  }

  if(veh_indexes[2] != -1) {
    vFrontVehCurrent = fix16_to_float(state[veh_lane][veh_indexes[2]].v);
    gapFrontCurrent = fix16_to_float(state[veh_lane][veh_indexes[2]].p - position) - veh_length;
  }

  if(veh_indexes[3] != -1) {
    vRearVehCurrent = fix16_to_float(state[veh_lane][veh_indexes[3]].v);
    gapRearCurrent = fix16_to_float(position - state[veh_lane][veh_indexes[3]].p) - veh_length;
  }


  if(veh_indexes[4] != -1) {
    vFrontVehRight = fix16_to_float(state[veh_lane + 1][veh_indexes[4]].v);
    gapFrontRight = fix16_to_float(state[veh_lane + 1][veh_indexes[4]].p - position) - veh_length;
  }

  if(veh_indexes[5] != -1) {
    vRearVehRight = fix16_to_float(state[veh_lane + 1][veh_indexes[5]].v);
    gapRearRight = fix16_to_float(position - state[veh_lane + 1][veh_indexes[5]].p) - veh_length;
  }

  float ownAccBefore = idm(vCurrent, vFrontVehCurrent - vCurrent, gapFrontCurrent);
  float ownAccLeftAfter = idm(vCurrent, vFrontVehLeft - vCurrent, gapFrontLeft);
  float ownAccRightAfter = idm(vCurrent, vFrontVehRight - vCurrent, gapFrontRight);

  float followerAccCurrBefore = idm(vRearVehCurrent, vCurrent - vRearVehCurrent, gapRearCurrent);
  float followerAccCurrAfter = idm(vRearVehCurrent, vCurrent - vFrontVehCurrent, gapRearCurrent + gapFrontCurrent + veh_length);

  float followerAccLeftBefore = veh_indexes[1] != -1 ? idm(vRearVehLeft, vFrontVehLeft - vRearVehLeft, gapRearLeft + gapFrontLeft + veh_length) : 3.0;
  float followerAccLeftAfter = veh_indexes[1] != -1 ? idm(vRearVehLeft, vCurrent - vRearVehLeft, gapRearLeft) : 3.0;

  float followerAccRightBefore = veh_indexes[5] != -1 ? idm(vRearVehRight, vFrontVehRight - vRearVehRight, gapRearRight + gapFrontRight + veh_length) : 3.0;
  float followerAccRightAfter = veh_indexes[5] != -1 ? idm(vRearVehRight, vCurrent - vRearVehRight, gapRearRight) : 3.0;

  float incentiveLeft = ownAccLeftAfter - ownAccBefore + politeness *
                        (followerAccLeftAfter - followerAccLeftBefore +
                         followerAccCurrAfter - followerAccCurrBefore);

  float incentiveRight = ownAccRightAfter - ownAccBefore + politeness *
                         (followerAccRightAfter - followerAccRightBefore +
                          followerAccCurrAfter - followerAccCurrBefore);


  bool want_left = (gapFrontLeft > 0 && gapRearLeft > 0 && veh_lane > 0 && followerAccLeftAfter >= safe_decel && incentiveLeft > incThreshold);
  bool want_right = (gapFrontRight > 0 && gapRearRight > 0 && veh_lane < state.size() - 1 && followerAccRightAfter >= safe_decel && incentiveRight > incThreshold);

  bool prefer_left = true;

  if(want_left && (!want_right || (incentiveLeft == incentiveRight && prefer_left) || incentiveLeft > incentiveRight)) {
    return -1;
  } else if(want_right) {
    return 1;
  }

  return 0;
}
