/* Udacity Self Driving Car Nano Degree
 * Project 5: Model Predictive Controller
 * Submitted by: Omer Waseem
 * Date: Mar 6th, 2018
 *
 * Filename: MPC.hpp
 * Description: header file for MPC.cpp
 *
 * References: the code below is based on Udacity's Model Predictive Controller lessons
 */

#ifndef MPC_HPP
#define MPC_HPP

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// Set the timestep length and duration
const size_t N = 10;
const double dt = 0.05;

// desired velocity for the cost optimization
const double ref_v = 60;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Structure to return desired values from MPC::Solve()
struct MPC_Result {
	vector<double> ptsx;
	vector<double> ptsy;
	double delta;
	double a;
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  MPC_Result Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_HPP */
