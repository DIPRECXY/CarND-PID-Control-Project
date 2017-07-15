#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  
  /*
  * Update Coefficients
  */  
  double dKp;
  double dKi;
  double dKd;
  
  double update_threshold;
  int update_position = 0;
  int n_step = 0;
  bool is_first_loop = true;
  double best_error;
  double current_error;
  double total_error;
  
  /*
  * Position Parameters
  */ 
  double x_trajectory = 0;
  double y_trajectory = 0;
  double orientation = 0;
  double steering_noise = 0;
  double distance_noise = 0;
  double steering_drift = 0;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  /*
  * Update coefficients by twiddle algorithm
  */
  void UpdateCoefficients();
  
  /*
  * Update throttle based on steering angle
  */
  double UpdateThrottle(double steer_value);
  
};

#endif /* PID_H */
