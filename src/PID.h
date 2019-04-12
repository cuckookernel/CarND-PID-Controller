#ifndef PID_H
#define PID_H

#include <tuple>

typedef std::tuple<double, double, double> triple;

class PID {
 public:
  /**
   * Constructor
   */
  PID(double Kp_, double Ki_, double Kd_);

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Set(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  triple ErrorParts();
  

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  double SteerValue();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error; 

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
};

#endif  // PID_H