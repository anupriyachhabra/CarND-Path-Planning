//
// Created by Anupriya Chhabra on 7/31/17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Vehicle {
public:

  int lane;

  double s;

  double vx;

  double vy;

  int a;

  /**
  * Constructor
  */
  Vehicle(int lane, int s, int v, int a);

  Vehicle(int lane, double s, double d, double vx, double vy, int a);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void update(int lane, double s, double d, double vx, double vy, int a);

  string display();

};


#endif //PATH_PLANNING_VEHICLE_H
