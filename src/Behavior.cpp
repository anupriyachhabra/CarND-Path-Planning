//
// Created by Anupriya Chhabra on 7/31/17.
//

#include "Behavior.h"


vector<vector<double>> Behavior::planRoute(Road road, vector<double> car_state,
                                           vector<double> previous_path_x, vector<double> previous_path_y,
                                           double end_path_s) {

  tgtr.road = road;
  predictor.road = road;
  predictor.path_size = previous_path_x.size();
  predictor.end_path_s = end_path_s;
  predictor.all_lane_collision = false;

  int current_car_lane = car_state[6];

  this->lane = next_lane(car_state);
  this->ref_vel = next_velocity(road);

  cout << "lane " << lane << endl;
  cout << "ref_vel " << ref_vel << endl;


  vector<vector<double>> trajectory = tgtr.generateTrajectories(car_state, previous_path_x, previous_path_y, lane,
                                                                ref_vel);

  return trajectory;
}

int Behavior::next_lane(vector<double> car_state) {
  int cur_lane = car_state[6];
  int lane = cur_lane;

  cout << "current_lane " << cur_lane << endl;
  bool collision = predictor.will_collide(cur_lane, car_state);
  cout << "collision in lane " << lane << " " << collision << endl;
  if (collision) {
    lane = 0;
    while (lane < 3 && collision) {
      if (lane != cur_lane) {
        collision = predictor.will_collide(lane, car_state);
        cout << "collision in lane " << lane << " " << collision << endl;
        if (!collision) break;
      }
      lane++;
    }
  }

  // If we couldnt find any lane where there is no collision then set all_collision to true so that speed is reduced
  // and keep driving in current lane
  // Also we dont want to change more than 1 lane at a time
  if (collision || abs(lane-cur_lane) > 1 ) {
    predictor.all_lane_collision = true;
    lane = cur_lane;
  }
  return lane;
}

double Behavior::next_velocity(Road road) {
  double velocity = ref_vel;
  if (predictor.all_lane_collision && velocity > 1.0) {
    velocity -= 0.224;
  } else if (velocity < road.speed_limit){
    velocity += 0.224;
  }
  return velocity;
}
