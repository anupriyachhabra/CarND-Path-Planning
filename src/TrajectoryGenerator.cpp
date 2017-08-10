//
// Created by Anupriya Chhabra on 7/31/17.
//

#include "TrajectoryGenerator.h"

vector<vector<double>> TrajectoryGenerator::generateTrajectories(vector<double> car_state, vector<double> previous_path_x,
                                                                 vector<double> previous_path_y, int target_lane) {
  //lets have 3 values for next car_d (L, R, C)
  vector<double> next_d = {2.0, 6.0, 10.0};
  double timestep = 0.2;

  vector<vector<double>> trajectory;

  double ref_x = car_state[0];
  double ref_y = car_state[1];
  double ref_yaw = helper.deg2rad(car_state[4]);
  int prev_size = previous_path_x.size();

  vector<double> ptsx;
  vector<double> ptsy;

  if (prev_size < 2) {
    // As previous path is very small we try to generate points by going back in time
    // We need to generate points as spline needs minimum of 3 points to function properly
    double prev_car_x = car_state[0] - cos(car_state[4]);
    double prev_car_y = car_state[1] - sin(car_state[4]);
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_state[0]);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_state[1]);

  } else {
    // use previous paths end point as reference if it is present, this is better for creating smoother trajectories
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];

    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw =  atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);

  }

  //ADD more points in frenet spaced 30 m apart
  for (int i =0; i <3 ; i ++) {
    vector<double> next_point = helper.getXY(car_state[2]+30*(i+1), 2+4*(target_lane) ,
                                             road.map_waypoints_s, road.map_waypoints_x, road.map_waypoints_y);
    ptsx.push_back(next_point[0]);
    ptsy.push_back(next_point[1]);
  }


  return vector<vector<double>>();
}
