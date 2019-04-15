#include <cmath>
#include <uWS/uWS.h>
#include <ostream>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "TwiddleOpt.h"
#include <iomanip>
#include <vector>
#include <numeric>

// for convenience
using nlohmann::json;
using namespace std;
using std::ostream;
using std::pair;

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

  int msg_cnt = 0;
  double integ_abs_cte = 0.0;
  double integ_cte = 0.0;
  double integ_speed = 0.0;

  TwiddleOptimizer opt;
  
  State( const vector<double>& init_pars, const vector<double> init_dp ) : opt( init_pars, init_dp ) {};

  void Reset() { msg_cnt = 0; integ_abs_cte = 0; integ_speed = 0.0; };
  double AvgSpeed() const { return integ_speed / msg_cnt; }
  double AvgAbsCte() const { return integ_abs_cte / msg_cnt; }
  double AvgCte() const { return integ_cte /msg_cnt; }
  double DistProxy() const { return integ_speed * 0.02; }
};

inline void final_output( ostream& os, const State& state, const tuple<double,double,double> pars, double throttle ) {
  os << std::fixed << std::setprecision(7) << get<0>(pars) << "\t" << get<1>(pars) << "\t" << get<2>(pars) 
          << "\t" << throttle 
          << "\t" << state.msg_cnt << "\t" << state.AvgAbsCte() << "\t" << state.AvgCte()
          << "\t" << state.DistProxy() << "\t" << state.AvgSpeed() << endl;
    
}

pair<double, double>  process(  PID &pid, State &state, double throttle ) {
  state.integ_abs_cte += fabs( state.cte );
  state.integ_speed += state.speed;
  
  if( fabs( state.cte ) > 3.0   ) {
    auto err_parts = pid.ErrorParts();
    state.integ_cte = get<1>( err_parts );
    final_output( cout, state, pid.Pars(), throttle );
    final_output( cerr, state, pid.Pars(), throttle );
    exit(1);
  }
  if( state.DistProxy() >= 10000 ) {
    
    final_output( cout, state, pid.Pars(), throttle );
    final_output( cerr, state, pid.Pars(), throttle );
    
    double err = state.AvgAbsCte() / state.AvgSpeed() ;
    auto pars = state.opt.eval_result( err );

    state.Reset();
    
    pid.Set( pars[0], pars[1], pars[2] ); 

    if( state.opt.IsDone() ) {
      cerr << " Done! " << state.opt.BestPars() << " best err " << state.opt.BestErr() << endl; 
      exit(1);
    }

  } 

  state.msg_cnt++;
  //double angle = std::stod(j[1]["steering_angle"].get<string>());
  pid.UpdateError( state.cte );
  double steer_value = pid.SteerValue() ;
  
  auto err_parts = pid.ErrorParts();

  if( VERBOSE ) {
    cerr  << std:: fixed <<  std::setprecision(4) 
          << state.msg_cnt << " speed: " << state.speed << " CTE: " << state.cte << " Steering Value: " << steer_value 
          << " errors: " << get<0>( err_parts ) << " " << get<1>( err_parts ) << " "<< get<2>( err_parts )
          << " dist_proxy: " << state.DistProxy()
          << std::endl;
  }

  double cte_dt = get<2>( err_parts );

  double out_throttle = throttle * std::max( 1 - (state.cte * state.cte) / 4.5  -cte_dt * cte_dt / 0.04  , -1.0 );
  return std::make_pair(steer_value, out_throttle );
}

int main(int narg, char **argv) {

  Params pars0;

  assert( narg == 5 );
  
  pars0.kp = atof( argv[1] );
  pars0.ki = atof( argv[2] );
  pars0.kd = atof( argv[3] );
  pars0.throttle = atof( argv[4] );
  double max_throttle = pars0.throttle;

  vector<double> init_pars{ pars0.kp, pars0.ki, pars0.kd};
  vector<double> init_dp{ 0.01, 0.00001, 0.1 };

  State state( init_pars, init_dp );

  uWS::Hub h;

  //PID pid(0.02, 0.0004, 0.03); // order is p, i, d
  //PID pid(0.04, 0.000, 0.00); 
  //PID pid(0.10, 0.000, 0.001); // 4000 steps
  //PID pid(0.02, 0.000, 0.001); // 1586 steps
  auto pars = state.opt.eval_result( NaN );

  PID pid(pars[0], pars[1], pars[2] ); // 1586 steps

  /**
   * TODO: Initialize the pid variable.
   */ 

  h.onMessage([ &pid, &state, max_throttle ]
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
          
          auto result = process(  pid, state, max_throttle );
          
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