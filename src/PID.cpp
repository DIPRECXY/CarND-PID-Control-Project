#include <iostream>
#include <math.h>
#include "PID.h"

using namespace std;

PID::PID() {
	_x_trajectory = 0;
	_y_trajectory = 0;
	_orientation = 0;
	_steering_noise = 0;
	_distance_noise = 0;
	_steering_drift = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	_Kp = Kp;
	_Ki = Ki;
	_Kd = Kd;

	_dKp = Kp==0? 0.1:Kp/10.;
	_dKi = Ki==0? 0.001:Ki/10.;
	_dKd = Kd==0? 1.0:Kd/10.;

	_update_threshold = 0.00001;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	i_error += cte;
	p_error = cte;

	_total_error += cte*cte;
	_n_step += 1;
}

double PID::UpdateThrottle(double steer_value) {

	double throttle = 0.75 - fabs(steer_value)*1.75;
	return throttle;
}

double PID::TotalError() {

	UpdateCoefficients();

	double steer_value = -_Kp*p_error - _Ki*i_error - _Kd*d_error;

	steer_value = steer_value >  1?  1:steer_value;
	steer_value = steer_value < -1? -1:steer_value;

	return steer_value;
}

void PID::UpdateCoefficients() {

	if ((_dKp + _dKi + _dKd) > _update_threshold) {
		_current_error = _total_error / _n_step;

		if (_is_first_loop) {
			// first loop of Twiddle
			if (_current_error < _best_error) {
				_best_error = _current_error;
				if (_update_position == 0) _dKp *= 1.1;
				if (_update_position == 1) _dKi *= 1.1;
				if (_update_position == 2) _dKd *= 1.1;
			} else {
				if (_update_position == 0) _Kp -= 2 * _dKp;
				if (_update_position == 1) _Ki -= 2 * _dKi;
				if (_update_position == 2) _Kd -= 2 * _dKd;
				_is_first_loop = false;
				_update_position += 1;
				_update_position = _update_position % 3;
				return;
			}
		} else {
			// second loop of Twiddle
			if (_current_error < _best_error) {
				_best_error = _current_error;
				if (_update_position == 0) _dKp *= 1.1;
				if (_update_position == 1) _dKi *= 1.1;
				if (_update_position == 2) _dKd *= 1.1;
			} else {
				if (_update_position == 0) {
					_Kp += _dKp;
					_dKp *= 0.9;
				}
				if (_update_position == 1) {
					_Ki += _dKi;
					_dKi *= 0.9;
				}
				if (_update_position == 2) {
					_Kd += _dKd;
					_dKd *= 0.9;
				}
				_is_first_loop = true;
			}
			if (_update_position == 0) _Kp += _dKp;
			if (_update_position == 1) _Ki += _dKi;
			if (_update_position == 2) _Kd += _dKd;
			_update_position += 1;
			_update_position = _update_position % 3;
		}
	}
}
