#include <algorithm>

#include "PID.h"

using std::min;
using std::max;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID(double Kp_, double Ki_, double Kd_) : Kp(Kp_), Ki(Ki_), Kd(Kd_) {}

PID::~PID() {}

void PID::Set(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd  = Kd_;
}

void PID::UpdateError(double cte) {
  double prev_cte = p_error;

  p_error = cte;
  i_error += cte;
  d_error = cte - prev_cte;

}

double PID::TotalError() {

  return Kp * p_error + Ki * i_error + Kd * d_error;  
}

triple PID::ErrorParts() {
  return std::make_tuple( Kp * p_error, Ki * i_error, Kd * d_error );
}

double PID::SteerValue() {
  // truncate total error to interval [-1.0, 1.0]
  return min( max( -TotalError(), -1.0 ), 1.0 );
}