//
// Created by Anupriya Chhabra on 7/31/17.
//

#ifndef PATH_PLANNING_BEHAVIOR_H
#define PATH_PLANNING_BEHAVIOR_H


#include "hw/road.h"
#include <vector>


class Behavior {

public:
  vector<vector<double>> planRoute(Road road, vector<double> car_state, double target_s);

};


#endif //PATH_PLANNING_BEHAVIOR_H
