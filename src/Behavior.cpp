//
// Created by Anupriya Chhabra on 7/31/17.
//

#include "Behavior.h"


vector<vector<double>> Behavior::planRoute(Road road, vector<double> car_state,
                                           vector<double> previous_path_x, vector<double> previous_path_y) {

  tgtr.road = road;
  vector<vector<double>> trajectory = tgtr.generateTrajectories(car_state, previous_path_x, previous_path_y, lane);
  // or there can be multiple trajectories above to choose from
  return trajectory;
}
