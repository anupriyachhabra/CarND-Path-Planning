//
// Created by Anupriya Chhabra on 7/31/17.
//

#ifndef PATH_PLANNING_BEHAVIOR_H
#define PATH_PLANNING_BEHAVIOR_H


#include "hw/road.h"
#include "TrajectoryGenerator.h"
#include "Prediction.h"
#include <vector>


class Behavior {
  TrajectoryGenerator tgtr;
  Prediction predictor;

public:
  vector<vector<double>> planRoute(Road road, vector<double> car_state,
                                   vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s);
  // lanes are 0, 1, 2
  int lane = 1;
  //
  double ref_vel = 0.5; //mph

  int next_lane(vector<double> vector);

  double next_velocity(Road road, bool is_lane_change);
};


#endif //PATH_PLANNING_BEHAVIOR_H
