#include "PID.h"
#include <iostream>
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  x_trajectory = 0;
  y_trajectory = 0;
  orientation = 0;
  steering_noise = 0;
  distance_noise = 0;
  steering_drift = 0;
  
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  dKp = Kp==0? 0.1:Kp/10.;
  dKi = Kp==0? 0.001:Ki/10.;
  dKd = Kp==0? 1.0:Kd/10.;
  
  update_threshold = 0.00001;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  i_error += cte;
  p_error = cte;
  
  total_error += cte*cte;
  n_step += 1;
}

double PID::UpdateThrottle(double steer_value) {
  double throttle = 0.75 - fabs(steer_value)*1.75;
  return throttle;
}

double PID::TotalError() {
  
  UpdateCoefficients();
  
  double steer_value = -Kp*p_error - Ki*i_error - Kd*d_error;
  
  steer_value = steer_value >  1?  1:steer_value;
  steer_value = steer_value < -1? -1:steer_value;
  
  return steer_value;
}

void PID::UpdateCoefficients() {
  
  if ((dKp + dKi + dKd) > update_threshold) {
    current_error = total_error / n_step;
    
    if (is_first_loop) {
      // first loop
      if (current_error < best_error) {
        best_error = current_error;
        if (update_position == 0) dKp *= 1.1;
        if (update_position == 1) dKi *= 1.1;
        if (update_position == 2) dKd *= 1.1;
      } else {
        if (update_position == 0) Kp -= 2 * dKp;
        if (update_position == 1) Ki -= 2 * dKi;
        if (update_position == 2) Kd -= 2 * dKd;
        is_first_loop = false;
        update_position += 1;
        update_position = update_position % 3;
        return;
      }
    } else {
        // second loop 
        if (current_error < best_error) {
          best_error = current_error;
          if (update_position == 0) dKp *= 1.1;
          if (update_position == 1) dKi *= 1.1;
          if (update_position == 2) dKd *= 1.1;
      } else {
          if (update_position == 0) {
            Kp += dKp;
            dKp *= 0.9;
          }
          if (update_position == 1) {
            Ki += dKi;
            dKi *= 0.9;
          }
          if (update_position == 2) {
            Kd += dKd;
            dKd *= 0.9;
          }
          is_first_loop = true;
        }
      if (update_position == 0) Kp += dKp;
      if (update_position == 1) Ki += dKi;
      if (update_position == 2) Kd += dKd;
      update_position +=1;
      update_position = update_position % 3;
    }
  }
}

