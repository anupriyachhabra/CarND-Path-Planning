//
// Created by Anupriya Chhabra on 7/31/17.
//

#include <iostream>
#include "road.h"
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>


/**
 * Initializes Road
 */

Road::Road() {

}
Road::~Road() {}

void Road::advance() {

  map<int ,vector<vector<int> > > predictions;

  map<int, Vehicle>::iterator it = this->vehicles.begin();
  while(it != this->vehicles.end())
  {
    int v_id = it->first;
    vector<vector<int> > preds = it->second.generate_predictions(10);
    predictions[v_id] = preds;
    it++;
  }
  it = this->vehicles.begin();
  while(it != this->vehicles.end())
  {
    int v_id = it->first;
    if(v_id == ego_key)
    {
      it->second.update_state(predictions);
      it->second.realize_state(predictions);
    }
    it->second.increment(1);

    it++;
  }

}

void Road::display(int timestep) {

 /* Vehicle ego = this->vehicles.find(this->ego_key)->second;
  int s = ego.s;
  string state = ego.state;

  this->camera_center = max(s, this->update_width/2);
  int s_min = max(this->camera_center - this->update_width/2, 0);
  int s_max = s_min + this->update_width;

  vector<vector<string> > road;

  for(int i = 0; i < this->update_width; i++)
  {
    vector<string> road_lane;
    for(int ln = 0; ln < this->num_lanes; ln++)
    {
      road_lane.push_back("     ");
    }
    road.push_back(road_lane);

  }

  map<int, Vehicle>::iterator it = this->vehicles.begin();
  while(it != this->vehicles.end())
  {

    int v_id = it->first;
    Vehicle v = it->second;

    if(s_min <= v.s && v.s < s_max)
    {
      string marker = "";
      if(v_id == this->ego_key)
      {
        marker = this->ego_rep;
      }
      else
      {

        stringstream oss;
        stringstream buffer;
        buffer << " ";
        oss << v_id;
        for(int buffer_i = oss.str().length(); buffer_i < 3; buffer_i++)
        {
          buffer << "0";

        }
        buffer << oss.str() << " ";
        marker = buffer.str();
      }
      road[int(v.s - s_min)][int(v.lane)] = marker;
    }
    it++;
  }
  ostringstream oss;
  oss << "+Meters ======================+ step: " << timestep << endl;
  int i = s_min;
  for(int lj = 0; lj < road.size(); lj++)
  {
    if(i%20 ==0)
    {
      stringstream buffer;
      stringstream dis;
      dis << i;
      for(int buffer_i = dis.str().length(); buffer_i < 3; buffer_i++)
      {
        buffer << "0";
      }

      oss << buffer.str() << dis.str() << " - ";
    }
    else
    {
      oss << "      ";
    }
    i++;
    for(int li = 0; li < road[0].size(); li++)
    {
      oss << "|" << road[lj][li];
    }
    oss << "|";
    oss << "\n";
  }

  cout << oss.str();
*/
}

