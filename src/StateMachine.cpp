//
// Created by Anupriya Chhabra on 7/31/17.
//

#include "StateMachine.h"


StateMachine::State
StateMachine::evaluate_next_state(StateMachine::State cur_state, int target_lane, int cur_lane, bool all_collision, double vel) {
  State state = KL;
  switch (cur_state) {
    case KL:
      if (target_lane == cur_lane) state = KL;
      else if (target_lane < cur_lane) state = PLCL;
      else if (target_lane > cur_lane) state = PLCR;
      break;
    case PLCL:
      if ((target_lane < cur_lane)) {
        if (all_collision || vel > 37.0) state = PLCL;
        else state = LCL;
      }
      break;
    case LCL:
      state = KL;
      break;
    case PLCR:
      if ((target_lane > cur_lane)) {
        if (all_collision || vel > 37.0) state = PLCR;
        else state = LCR;
      }
      break;
    case LCR:
      state = KL;
      break;
    default:
      state = cur_state;
      break;


  }
  return state;
}
