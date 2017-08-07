#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include <time.h>

// for convenience
using json = nlohmann::json;

using namespace std;
using namespace Eigen;

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

void get_rotation_matrix(double psi, double px, double py, Eigen::MatrixXd &rotation_matrx){
  // Initialize rotation matrix with Identity matrix
  // rotation_matrx(0,0) = rotation_matrx(1,1) = 10;
  rotation_matrx(0,0) = rotation_matrx(1,1) = cos(psi);
  rotation_matrx(0,1) = -sin(psi);
  rotation_matrx(1,0) = sin(psi);
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  // Temporary values to mitigate latency
  double _acc = 0.0;
  double _ste = 0.0;
  auto start_time = std::chrono::system_clock::now();

  Eigen::MatrixXd rotation_matrx(2,2);
  VectorXd translation(2);
  VectorXd state_vector(6);
  Eigen::MatrixXd global_traj;
  vector<double> results;

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = -1.0 * double(j[1]["psi"]); // Get data in right hand coord
          double v = 0.44704 * double(j[1]["speed"]); // Convert mph to m/s
          // Collect additional parameters from telemetry data:
          double _t_steer = j[1]["steering_angle"];
          double _t_throt = j[1]["throttle"];

          // Translate and rotate vector to local coordinates
          translation << px, py;
          rotation_matrx << cos(psi), -sin(psi),
                            sin(psi),  cos(psi);
          assert(ptsx.size() == ptsy.size());
          global_traj.resize(2,ptsx.size());
          global_traj.row(0) = Eigen::Map<Eigen::VectorXd>(ptsx.data(), ptsx.size());
          global_traj.row(1) = Eigen::Map<Eigen::VectorXd>(ptsy.data(), ptsy.size());
          MatrixXd local_traj = rotation_matrx * (global_traj.colwise() - translation);

          // Fit 3rd order polynomial to trajectory
          VectorXd coeffs = polyfit(local_traj.row(0), local_traj.row(1), 3);

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          auto end_time = std::chrono::system_clock::now();
          std::chrono::duration<double> latency =  end_time - start_time;
          std::cout << "Delay: " << latency.count() << std::endl;
          // Fill the state vector:
          double _x = v * cos(psi) * latency.count();
          double _y = v * sin(psi) * latency.count();
          double _psi = -v * _t_steer * latency.count() / mpc.Lf;
          double _cte = polyeval(coeffs, _x);
          double _epsi = _psi - atan2(polyeval(coeffs, _x), _x);
          double _v = v;

          state_vector << _x, _y, _psi, _v, _cte, _epsi;
          // cout << state_vector << endl;

          results = mpc.Solve(state_vector, coeffs);

          double steer_value =  - 1.0 * results[0]/DELTA_THRESHOLD;
          double throttle_value = results[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          size_t _len = (results.size()-2)/2;
          for(int i=2; i<_len;i++){
            mpc_x_vals.push_back(results[i]);
            mpc_y_vals.push_back(results[i + _len]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for(int i=0; i<10; ++i){
              next_x_vals.push_back(i);
              next_y_vals.push_back(polyeval(coeffs, i));
          }
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;

          start_time = std::chrono::system_clock::now();  // Record time
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
