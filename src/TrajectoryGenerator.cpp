//
// Created by Anupriya Chhabra on 7/31/17.
//

#include "TrajectoryGenerator.h"
#include "utils/spline.h"

vector<vector<double>> TrajectoryGenerator::generateTrajectories(vector<double> car_state, vector<double> previous_path_x,
                                                                 vector<double> previous_path_y, int target_lane,
                                                                 double target_vel) {
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

  // transform coordinates to vehicle's to make the math easier
  helper.transformToVehicleCoord(ptsx, ptsy, ref_x, ref_y, ref_yaw);

  //Generate Path
  // Add previous_path for smoothing- use only 10 previous points

  cout << "previous path size " << previous_path_x.size() << endl;
  int previous_points = min((int)previous_path_x.size(), 50);
  for (int i = 0; i < previous_points; i++){

    vector<double> path { previous_path_x[i], previous_path_y[i] };
    trajectory.push_back(path);
  }

  tk::spline s;
  s.set_points(ptsx, ptsy);

  // Break spline points so that we travel at reference velocity
  // We chooose 30m as target below to calculate at some not so far distance in future
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

  double x_add_on = 0;

  double vel_increment = (target_vel - car_state[5])/(50- previous_points);
  double vel = car_state[5] + vel_increment;
  //fill up rest of the trajectory making sure that the new generated points dont make the target_velocity go high
  for (int i = 0; i < 50-previous_points; i++) {

    double N = (target_dist/(0.02*vel/2.24));
    double x_point = x_add_on+(target_x)/N;
    double y_point = s(x_point);

    x_add_on = x_point; // we want future points to be greater than the point already generated

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate to vehicle coordinates before adding to trajectory
    x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    vector<double> path { x_point, y_point };
    //cout << "x_point" << x_point << endl;
    trajectory.push_back(path);

    vel += vel_increment;
    if (vel > road.speed_limit) vel = road.speed_limit;
  }

  return trajectory;
}
