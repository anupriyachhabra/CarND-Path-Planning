//
// Created by Anupriya Chhabra on 7/31/17.
//

#include "TrajectoryGenerator.h"

vector<vector<double>> TrajectoryGenerator::generateTrajectories(vector<double> start_s, vector<double> start_d,
                                                                 double target_s, double target_d,
                                                                 int num_steps) {
  //lets have 3 values for next car_d (L, R, C)
  vector<double> next_d = {2.0, 6.0, 10.0};
  double timestep = 0.2;

  return vector<vector<double>>();
}
