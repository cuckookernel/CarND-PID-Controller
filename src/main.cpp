#include <cmath>
#include <uWS/uWS.h>
#include <ostream>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <iomanip>


// for convenience
using nlohmann::json;
using namespace std;
using std::ostream;

constexpr bool VERBOSE = false;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

struct Params {
  double kp, ki, kd, throttle;
  double smooth_alpha = 0.0; 
};

struct State {
  double cte;
  double speed;

  double smooth_steer = 0;

  int msg_cnt;
  double integ_abs_cte = 0.0 ;
  double integ_cte = 0.0;
  double integ_speed = 0.0 ;
};

using std::pair;

inline void final_output( ostream& os, const State& state, const Params& pars ) {
  os << std::fixed << std::setprecision(7) << pars.kp << "\t" << pars.ki << "\t" << pars.kd << "\t" << pars.throttle 
          << "\t" << state.msg_cnt << "\t" << state.integ_abs_cte / state.msg_cnt << "\t" << state.integ_cte / state.msg_cnt 
          << "\t" << state.integ_speed * 0.02 << endl;
    
}

pair<double, double>  process( const Params& pars, PID &pid, State &state ) {
  state.integ_abs_cte += fabs( state.cte );
  state.integ_speed += state.speed;
  
  if( fabs( state.cte ) > 3.0  || state.msg_cnt >= 10000 ) {
    auto err_parts = pid.ErrorParts();
    state.integ_cte = get<1>( err_parts );
    final_output( cout, state, pars );
    final_output( cerr, state, pars );
    exit(1);
  }
  state.msg_cnt++;
  //double angle = std::stod(j[1]["steering_angle"].get<string>());
  pid.UpdateError( state.cte );
  double steer_value = pid.SteerValue() ;
  state.smooth_steer = state.smooth_steer * pars.smooth_alpha + ( 1.0 - pars.smooth_alpha ) * steer_value;
  
  auto err_parts = pid.ErrorParts();

  if( VERBOSE ) {
    cerr  << std:: fixed <<  std::setprecision(4) 
          << state.msg_cnt << " speed: " << state.speed << " CTE: " << state.cte << " Steering Value: " << state.smooth_steer 
          << " errors: " << get<0>( err_parts ) << " " << get<1>( err_parts ) << " "<< get<2>( err_parts )
          << " dist: " << state.integ_speed * 0.02
          << std::endl;
  }

  return std::make_pair(state.smooth_steer, pars.throttle );
}


int main(int narg, char **argv) {

  State state;
  Params pars;

  assert( narg == 5 );
  
  pars.kp = atof( argv[1] );
  pars.ki = atof( argv[2] );
  pars.kd = atof( argv[3] );
  pars.throttle = atof( argv[4] );

  uWS::Hub h;

  //PID pid(0.02, 0.0004, 0.03); // order is p, i, d
  //PID pid(0.04, 0.000, 0.00); 
  //PID pid(0.10, 0.000, 0.001); // 4000 steps
  //PID pid(0.02, 0.000, 0.001); // 1586 steps
  PID pid(pars.kp, pars.ki, pars.kd ); // 1586 steps

  /**
   * TODO: Initialize the pid variable.
   */ 

  h.onMessage([ &pid, &state, &pars]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();
        //if (msg_cnt < 1) { cerr << j.dump() << endl; }

        if (event == "telemetry") {
          // j[1] is the data JSON object
          state.cte = std::stod(j[1]["cte"].get<string>());
          state.speed = std::stod(j[1]["speed"].get<string>());
          
          auto result = process( pars, pid, state );
          
          json msgJson;
          msgJson["steering_angle"] = result.first;
          msgJson["throttle"] = result.second;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }

      
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cerr << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cerr << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cerr << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}