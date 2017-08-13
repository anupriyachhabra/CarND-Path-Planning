//
// Created by Anupriya Chhabra on 7/31/17.
//

#ifndef PATH_PLANNING_JMTGENERATOR_H
#define PATH_PLANNING_JMTGENERATOR_H
#include <vector>
#include "utils/helpers.h"
#include "hw/road.h"
#include <algorithm>

using namespace::std;

class TrajectoryGenerator {

public:
  vector<vector<double>> generateTrajectories(vector<double> car_state, vector<double> previous_path_x,
                                              vector<double> previous_path_y, int target_lane, double target_vel);
  helpers helper;
  Road road;

};


#endif //PATH_PLANNING_JMTGENERATOR_H
