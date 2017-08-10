//
// Created by Anupriya Chhabra on 7/31/17.
//

#ifndef PATH_PLANNING_BEHAVIOR_H
#define PATH_PLANNING_BEHAVIOR_H


#include "hw/road.h"
#include "TrajectoryGenerator.h"
#include <vector>


class Behavior {
  TrajectoryGenerator tgtr;

public:
  vector<vector<double>> planRoute(Road road, vector<double> car_state,
                                   vector<double> previous_path_x, vector<double> previous_path_y);
  // lanes are 0, 1, 2
  int lane = 1;
  //
  double ref_vel = 49.5; //mph

};


#endif //PATH_PLANNING_BEHAVIOR_H
