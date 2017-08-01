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
  state = "CS";
  max_acceleration = -1;

}

Vehicle::Vehicle(int lane, double s, double d, double vx, double vy, int a) {
  this->lane = lane;
  this->s = s;
  this->vx = vx;
  this->vy = vy;
  this->a = a;
  state = "CS";
  max_acceleration = -1;
}


Vehicle::~Vehicle() {}

void Vehicle::update(int lane, double s, double d, double vx, double vy, int a) {
  this->lane = lane;
  this->s = s;
  this->vx = vx;
  this->vy = vy;
  this->a = a;
  state = "CS";
  max_acceleration = -1;
}

// TODO - Implement this method.
void Vehicle::update_state(map<int,vector < vector<int> > > predictions) {
  /*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */
  // this is an example of how you change state.
  std::cout << "current lane" << lane << endl;
  if (lane ==goal_lane || lanes_available == 0)
    state = "KL";
  if (lane == goal_lane && (goal_s - s) < 15)
    state = "KL";
  else if (goal_lane - lane > 0)
    if(state == "PLCL" && lanes_available > 0)
      state = "LCL";
    else
      state = "PLCL";
  else if (goal_lane - lane < 0)
    if(state == "PLCR" && lanes_available > 0)
      state = "LCR";
    else
      state = "PLCR";
  if (vx < target_speed)
    max_acceleration = 2;
  else if (vx > target_speed)
    max_acceleration = -1;

  std::cout << "state" << state << endl;


}

void Vehicle::configure(vector<int> road_data) {
  /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
  target_speed = road_data[0];
  lanes_available = road_data[1];
  max_acceleration = road_data[2];
  goal_lane = road_data[3];
  goal_s = road_data[4];
}

string Vehicle::display() {

  ostringstream oss;

  oss << "s:    " << this->s << "\n";
  oss << "lane: " << this->lane << "\n";
  oss << "v:    " << this->vx << "\n";
  oss << "a:    " << this->a << "\n";

  return oss.str();
}

void Vehicle::increment(int dt = 1) {

  this->s += this->vx * dt;
  this->vx += this->a * dt;
}

vector<int> Vehicle::state_at(int t) {

  /*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
  int s = this->s + this->vx * t + this->a * t * t / 2;
  int v = this->vx + this->a * t;
  return {this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, int at_time) {

  /*
    Simple collision detection.
    */
  vector<int> check1 = state_at(at_time);
  vector<int> check2 = other.state_at(at_time);
  return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {

  Vehicle::collider collider_temp;
  collider_temp.collision = false;
  collider_temp.time = -1;

  for (int t = 0; t < timesteps+1; t++)
  {
    if( collides_with(other, t) )
    {
      collider_temp.collision = true;
      collider_temp.time = t;
      return collider_temp;
    }
  }

  return collider_temp;
}

void Vehicle::realize_state(map<int,vector < vector<int> > > predictions) {

  /*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
  string state = this->state;
  if(state.compare("CS") == 0)
  {
    realize_constant_speed();
  }
  else if(state.compare("KL") == 0)
  {
    realize_keep_lane(predictions);
  }
  else if(state.compare("LCL") == 0)
  {
    realize_lane_change(predictions, "L");
  }
  else if(state.compare("LCR") == 0)
  {
    realize_lane_change(predictions, "R");
  }
  else if(state.compare("PLCL") == 0)
  {
    realize_prep_lane_change(predictions, "L");
  }
  else if(state.compare("PLCR") == 0)
  {
    realize_prep_lane_change(predictions, "R");
  }

}

void Vehicle::realize_constant_speed() {
  a = 0;
}

int Vehicle::_max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s) {

int delta_v_til_target = target_speed - vx;
int max_acc = min(max_acceleration, delta_v_til_target);

map<int, vector<vector<int> > >::iterator it = predictions.begin();
vector<vector<vector<int> > > in_front;
while(it != predictions.end())
{

int v_id = it->first;

vector<vector<int> > v = it->second;

if((v[0][0] == lane) && (v[0][1] > s))
{
in_front.push_back(v);

}
it++;
}

if(in_front.size() > 0)
{
int min_s = 1000;
vector<vector<int>> leading = {};
for(int i = 0; i < in_front.size(); i++)
{
if((in_front[i][0][1]-s) < min_s)
{
min_s = (in_front[i][0][1]-s);
leading = in_front[i];
}
}

int next_pos = leading[1][1];
int my_next = s + this->vx;
int separation_next = next_pos - my_next;
int available_room = separation_next - preferred_buffer;
max_acc = min(max_acc, available_room);
}

return max_acc;

}

void Vehicle::realize_keep_lane(map<int,vector< vector<int> > > predictions) {
  this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int,vector< vector<int> > > predictions, string direction) {
int delta = -1;
if (direction.compare("L") == 0)
{
delta = 1;
}
this->lane += delta;
int lane = this->lane;
int s = this->s;
this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(map<int,vector<vector<int> > > predictions, string direction) {
int delta = -1;
if (direction.compare("L") == 0)
{
delta = 1;
}
int lane = this->lane + delta;

map<int, vector<vector<int> > >::iterator it = predictions.begin();
vector<vector<vector<int> > > at_behind;
while(it != predictions.end())
{
int v_id = it->first;
vector<vector<int> > v = it->second;

if((v[0][0] == lane) && (v[0][1] <= this->s))
{
at_behind.push_back(v);

}
it++;
}
if(at_behind.size() > 0)
{

int max_s = -1000;
vector<vector<int> > nearest_behind = {};
for(int i = 0; i < at_behind.size(); i++)
{
if((at_behind[i][0][1]) > max_s)
{
max_s = at_behind[i][0][1];
nearest_behind = at_behind[i];
}
}
int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
int delta_v = this->vx - target_vel;
int delta_s = this->s - nearest_behind[0][1];
if(delta_v != 0)
{

int time = -2 * delta_s/delta_v;
int a;
if (time == 0)
{
a = this->a;
}
else
{
a = delta_v/time;
}
if(a > this->max_acceleration)
{
a = this->max_acceleration;
}
if(a < -this->max_acceleration)
{
a = -this->max_acceleration;
}
this->a = a;
}
else
{
int my_min_acc = max(-this->max_acceleration,-delta_s);
this->a = my_min_acc;
}

}

}

vector<vector<int> > Vehicle::generate_predictions(int horizon = 10) {

  vector<vector<int> > predictions;
  for( int i = 0; i < horizon; i++)
  {
    vector<int> check1 = state_at(i);
    vector<int> lane_s = {check1[0], check1[1]};
    predictions.push_back(lane_s);
  }
  return predictions;

}

