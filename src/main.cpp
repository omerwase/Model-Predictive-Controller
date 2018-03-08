/* Udacity Self Driving Car Nano Degree
 * Project 5: Model Predictive Controller
 * Submitted by: Omer Waseem
 * Date: Mar 8th, 2018
 *
 * Filename: main.cpp
 * Description: implements the MPC algorithm. Recieves vehicle measurements from Udacity's
 *	 SDC Simulator, and sends back actuation commands based on MPC solution.
 *
 * References: the code below is based on Udacity's Model Predictive Controller lessons
 */

#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.hpp"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;
	
  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx_global = j[1]["ptsx"];
          vector<double> ptsy_global = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v0 = j[1]["speed"];
					double delta0 = j[1]["steering_angle"];
					double a0 = j[1]["throttle"];
					// Latency
					// The purpose is to mimic real driving conditions where
					// the car does actuate the commands instantly.
					const int delay_ms = 100;
					const double delay_s = delay_ms / 1000.0;
					// Number of reference points to consider for polyfit
					size_t look_ahead_pts = ptsx_global.size();

					// Transform reference trajector recieved from the simulator, from global
					// to vehicle coordinate system (vehicle is at origin)
					assert(ptsx_global.size() == ptsy_global.size());
					Eigen::VectorXd ptsx_v(look_ahead_pts);
					Eigen::VectorXd ptsy_v(look_ahead_pts);
					vector<double> next_x_vals;
					vector<double> next_y_vals;
					for (size_t i = 0; i < look_ahead_pts; ++i) {
						double dx = ptsx_global[i] - px;
						double dy = ptsy_global[i] - py;
						// ptsx_v and ptsy_v are Eigen vectors used for polyfit
						ptsx_v(i) = dx * cos(psi) + dy * sin(psi);
						ptsy_v(i) = dy * cos(psi) - dx * sin(psi);
						// next_x_vals and next_y_vals are std vectors used for simulator feed
						next_x_vals.push_back(ptsx_v(i));
						next_y_vals.push_back(ptsy_v(i));
					}
					
					// Fit transformed trajectory to a 3rd order ploynomial
					Eigen::VectorXd coeffs = polyfit(ptsx_v, ptsy_v, 3);
					
					// Calculate cross track error (cte) and psi error (epsi)
					// Equations are simplified since px = py = psi = 0 in vehicle coordinate system
					double cte0 = coeffs[0];
					double epsi0 = -atan(coeffs[1]);
					
					// Calculate new state based on actuate delay, assuming kinematic vehicle motion
					Eigen::VectorXd delayed_state(6);
					// Recalcuate the state of the vehicle considering latency effects
					// delayed_state is of the form [x y psi v cte epsi]
					// Equations are simplified since px = py = psi = 0 in vehicle coordinate system
					// Also simplied equations using cos(0) = 1 and sin(0) = 0
					double delayed_x = v0 * delay_s;
					double delayed_y = 0;
					double delayed_psi = -(v0 * delta0 * delay_s / Lf);
					double delayed_v = v0 + a0 * delay_s;
					double delayed_cte = cte0 + v0 * sin(epsi0) * delay_s;
					double delayed_epsi = epsi0 - v0 * delta0 + delay_s / Lf;
					delayed_state << delayed_x, delayed_y, delayed_psi, delayed_v, delayed_cte, delayed_epsi;
					
          // Calculate steering angle and throttle using MPC.
					MPC_Result res = mpc.Solve(delayed_state, coeffs);
					
					json msgJson;
					
					// Both steer_value and throttle_value should be between -1 and 1
					// steer_value is divided by deg2rad(25) to normalize value between -1 and 1
					double steer_value = res.delta/(deg2rad(25)*Lf);
          double throttle_value = res.a;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
					assert(res.ptsx.size() == res.ptsy.size());
					vector<double> mpc_x_vals = res.ptsx;
					vector<double> mpc_y_vals = res.ptsy;
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
					
          this_thread::sleep_for(chrono::milliseconds(delay_ms));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
