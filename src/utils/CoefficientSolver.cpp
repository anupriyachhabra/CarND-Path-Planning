//
// Created by Anupriya Chhabra on 7/31/17.
//

#include "CoefficientSolver.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include "../Eigen-3.3/Eigen/Core"
#include "../Eigen-3.3/Eigen/LU"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

vector<double> JMT(vector< double> start, vector <double> end, double T)
{
  /*
  Calculate the Jerk Minimizing Trajectory that connects the initial state
  to the final state in time T.

  INPUTS

  start - the vehicles start location given as a length three array
      corresponding to initial values of [s, s_dot, s_double_dot]

  end   - the desired end state for vehicle. Like "start" this is a
      length three array.

  T     - The duration, in seconds, over which this maneuver should occur.

  OUTPUT
  an array of length 6, each value corresponding to a coefficent in the polynomial
  s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

  EXAMPLE

  > JMT( [0, 10, 0], [10, 10, 0], 1)
  [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
  */
  Eigen::MatrixXd rhs = MatrixXd(3, 1);
  Eigen::MatrixXd lhs = MatrixXd(3, 3);

  rhs << end[0] - (start[0] + start[1]*T + start[2]*T*T/2),
      end[1] - (start[1] + start[2]*T),
      end[2] - start[2];

  double t2 = T*T;
  double t3 = t2*T;
  double t4 = t3*T;
  double t5 = t4*T;

  lhs << t3,  t4, t5,
      3*t2,    4*t3,   5*t4,
      6*T,     12*t2,  20*t3;

  Eigen::MatrixXd coeffs = lhs.inverse() * rhs;

  double a0 = start[0];
  double a1 = start[1];
  double a2 = start[2]/2;
  double a3 = coeffs.col(0)[0];
  double a4 = coeffs.col(0)[1];
  double a5 = coeffs.col(0)[2];

  return {a0, a1, a2, a3, a4, a5};

}