//
// Created by Anupriya Chhabra on 7/31/17.
//

#include "Prediction.h"

bool Prediction::will_collide(int lane, vector<double> car_state) {
  bool collision = false;
  double car_s = car_state[2];

  if(path_size > 0) {
    car_s = this->end_path_s;
  }

  map<int, Vehicle>::iterator it = this->road.vehicles.begin();
  while (it != this->road.vehicles.end() && !collision) {

    int v_id = it->first;
    Vehicle v = it->second;

    // we check only vehicles in a particular lane for collision
    if (v.lane == lane) {
      cout << "checking vehicle " << v_id << " in lane " << v.lane << endl;
      double vehicle_speed = sqrt(v.vx*v.vx + v.vy*v.vy);
      double vehicle_s = v.s;
      cout << "vehicle speed " << vehicle_speed << " vehicle s " << vehicle_s;

      // predicting future vehicle_s
      vehicle_s += ((double)path_size*0.02*vehicle_speed);

      cout << " future vehicle s " << vehicle_s << " future us s " << car_s << endl;
      //now check if future vehicle s will be greater than car_s and if distance between them is too less
      if (((vehicle_s > car_s) && ((vehicle_s - car_s) < 30)) || fabs(vehicle_s - car_s) < 5.0) {
        collision = true;
        cout << "collision true";
        break;
      }
    }
    it++;
  }

  return collision;
}
