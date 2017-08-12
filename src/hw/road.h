//
// Created by Anupriya Chhabra on 7/31/17.
//

#ifndef PATH_PLANNING_ROAD_H
#define PATH_PLANNING_ROAD_H


#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"

using namespace std;

class Road {
public:

  Road();

  int ego_key = -1;

  map<int, Vehicle> vehicles;

  int vehicles_added = 0;

  /**
  * Destructor
  */
  virtual ~Road();

  void advance();

  void display(int timestep);

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;

  double speed_limit = 49.0; //mph;

};


#endif //PATH_PLANNING_ROAD_H
