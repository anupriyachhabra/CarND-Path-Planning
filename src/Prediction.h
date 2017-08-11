//
// Created by Anupriya Chhabra on 7/31/17.
//

#ifndef PATH_PLANNING_PREDICTION_H
#define PATH_PLANNING_PREDICTION_H

#include <vector>
#include "hw/road.h"

using namespace::std;

class Prediction {

public:
  bool will_collide(int lane, vector<double> car_state);

  Road road;
  bool all_lane_collision = false;
  int path_size = 50;
  double end_path_s;

};


#endif //PATH_PLANNING_PREDICTION_H
