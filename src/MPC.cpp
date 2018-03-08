/* Udacity Self Driving Car Nano Degree
 * Project 5: Model Predictive Controller
 * Submitted by: Omer Waseem
 * Date: Mar 8th, 2018
 *
 * Filename: MPC.cpp
 * Description: defines vehicle model and constraints to optimize actuations based on
 *	 a reference trajectory. Uses ipopt and CppAD libraries.
 *
 * References: the code below is based on Udacity's Model Predictive Controller lessons
*/

#include "MPC.hpp"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// See MPC.hpp for hyperparameters

// start indices for fg and var vectors
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
	
	// `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
  void operator()(ADvector& fg, const ADvector& vars) {
		
		// Setup cost function
		// Cost is stored as the first element of fg
		// Cost is tuned using multipliers (first number after '+=')
		fg[0] = 0; // initialize to 0
		// Cost based on reference trajectory and velocity
		for (size_t i = 0; i < N; ++i) {
			fg[0] += 3000 * CppAD::pow(vars[cte_start + i], 2);
			fg[0] += 3000 * CppAD::pow(vars[epsi_start + i], 2);
			fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
		}
		// Cost based on magnitude of actuations, to avoid large values
		for (size_t i = 0; i < N - 1; ++i) {
			fg[0] += 5 * CppAD::pow(vars[delta_start + i], 2);
			fg[0] += CppAD::pow(vars[a_start + i], 2);
		}
		// Minimize gap between actuations, for smoother motion during state transition
		for (size_t i = 0; i < N - 2; ++i) {
			fg[0] += 500 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
			fg[0] += 5 * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
		}
		
		// Setup contraints based on vehicle motion (kinematic) model
		// Set initial state and error values
		// Note: 1 is added to fg index since fg[0] contains the cost
		fg[1 + x_start] = vars[x_start];
		fg[1 + y_start] = vars[y_start];
		fg[1 + psi_start] = vars[psi_start];
		fg[1 + v_start] = vars[v_start];
		fg[1 + cte_start] = vars[cte_start];
		fg[1 + epsi_start] = vars[epsi_start];
		// Set remaining constraints based on kinematic motion model
		
		// The rest of the constraints for the vehicle motion model
		for (int t = 1; t < N; t++) {
			AD<double> x1 = vars[x_start + t];
			AD<double> y1 = vars[y_start + t];
			AD<double> psi1 = vars[psi_start + t];
			AD<double> v1 = vars[v_start + t];
			AD<double> cte1 = vars[cte_start + t];
			AD<double> epsi1 = vars[epsi_start + t];
			
			AD<double> x0 = vars[x_start + t - 1];
			AD<double> y0 = vars[y_start + t - 1];
			AD<double> psi0 = vars[psi_start + t - 1];
			AD<double> v0 = vars[v_start + t - 1];
			AD<double> cte0 = vars[cte_start + t - 1];
			AD<double> epsi0 = vars[epsi_start + t - 1];
			
			AD<double> delta0 = vars[delta_start + t - 1];
			AD<double> a0 = vars[a_start + t - 1];
			
			AD<double> x0_2 = x0 * x0;
			AD<double> x0_3 = x0_2 * x0;
			AD<double> f0 = coeffs[3]*x0_3 + coeffs[2]*x0_2 + coeffs[1]*x0 + coeffs[0];
			// based on derivative of 3rd order polynomial c3*x^3 + c2*x^2 + c1*x + c0
			// which is 3*c3*x^2 + 2*c2*x + c1
			AD<double> psides0 = CppAD::atan(3*coeffs[3]*x0_2 + 2*coeffs[2]*x0 + coeffs[1]);
			
			fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
			fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 * dt / Lf);
			fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
			fg[1 + cte_start + t] = cte1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * dt);
			fg[1 + epsi_start + t] = epsi1 - (psi0 - psides0 - v0 * delta0 * dt / Lf);
		}
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

MPC_Result MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
	
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

	// current state variables
	double x = state[0];
	double y = state[1];
	double psi = state[2];
	double v = state[3];
	double cte = state[4];
	double epsi = state[5];
	
	// number of model variables (6 state variables at all N timesteps, and 2 actuator
	// variables for N-1 timesteps)
  size_t n_vars = N * 6 + (N - 1) * 2;
  size_t n_constraints = N * 6;

  // Initial value of the independent variables should be 0, besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }
	vars[x_start] = x;
	vars[y_start] = y;
	vars[psi_start] = psi;
	vars[v_start] = v;
	vars[cte_start] = cte;
	vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
	// Upper and lower bounds for state variables
	for (size_t i = 0; i < delta_start; ++i) {
		vars_lowerbound[i] = -1.0e19; // minimum possible value
		vars_upperbound[i] = 1.0e19; // maximum possible value
	}
	// Upper and lower bounds for actuator variables
	// -25 deg <= delta (steering angle) <= 25 degs
	for (size_t i = delta_start; i < a_start; ++i) {
		vars_lowerbound[i] = -0.436332*Lf; // in radians
		vars_upperbound[i] = 0.436332*Lf;
	}
	// -1 <= acceleration/decceleration <= 1
	for (size_t i = a_start; i < n_vars; ++i) {
		vars_lowerbound[i] = -1.0;
		vars_upperbound[i] = 1.0;
	}

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
	constraints_lowerbound[x_start] = x;
	constraints_lowerbound[y_start] = y;
	constraints_lowerbound[psi_start] = psi;
	constraints_lowerbound[v_start] = v;
	constraints_lowerbound[cte_start] = cte;
	constraints_lowerbound[epsi_start] = epsi;
	constraints_upperbound[x_start] = x;
	constraints_upperbound[y_start] = y;
	constraints_upperbound[psi_start] = psi;
	constraints_upperbound[v_start] = v;
	constraints_upperbound[cte_start] = cte;
	constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // The solution variables can be accessed with `solution.x[i]`.
	// result to return
	// contains current actuator commands to send
	MPC_Result res;
	// only need actuator values for the current state
	res.delta = solution.x[delta_start];
	res.a = solution.x[a_start];
	// x and y values of optimized trajector are used only for plotting
	for (size_t i = 0; i < N; ++i) {
		res.ptsx.push_back(solution.x[x_start + i]);
		res.ptsy.push_back(solution.x[y_start + i]);
	}
  return res;
}
