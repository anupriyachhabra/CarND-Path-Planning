//
// Created by Anupriya Chhabra on 7/31/17.
//

#ifndef PATH_PLANNING_JMTGENERATOR_H
#define PATH_PLANNING_JMTGENERATOR_H
#include <vector>

using namespace::std;

class TrajectoryGenerator {

public:
  vector<vector<double>> generateTrajectories(vector<double> start_s, vector<double> start_d, double target_s,
                                              double target_d, int num_steps);

};


#endif //PATH_PLANNING_JMTGENERATOR_H
