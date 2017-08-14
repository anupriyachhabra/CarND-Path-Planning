//
// Created by Anupriya Chhabra on 7/31/17.
//

#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int v, int a) {

  this->lane = lane;
  this->s = s;
  this->vx = v;
  this->a = a;
}

Vehicle::Vehicle(int lane, double s, double d, double vx, double vy, int a) {
  this->lane = lane;
  this->s = s;
  this->vx = vx;
  this->vy = vy;
  this->a = a;
}


Vehicle::~Vehicle() {}

void Vehicle::update(int lane, double s, double d, double vx, double vy, int a) {
  this->lane = lane;
  this->s = s;
  this->vx = vx;
  this->vy = vy;
  this->a = a;
}

string Vehicle::display() {

  ostringstream oss;

  oss << "s:    " << this->s << "\n";
  oss << "lane: " << this->lane << "\n";
  oss << "v:    " << this->vx << "\n";
  oss << "a:    " << this->a << "\n";

  return oss.str();
}

