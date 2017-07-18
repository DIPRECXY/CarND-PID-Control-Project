#include <iostream>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

const double PID::KP_DEFAULT = 0.23;
const double PID::KI_DEFAULT = 0.002;
const double PID::KD_DEFAULT = 14.0;
const double PID::INITIAL_ERROR = 0.0;

PID::PID() {
	p_error = PID::INITIAL_ERROR;
	i_error = PID::INITIAL_ERROR;
	d_error = PID::INITIAL_ERROR;
	Kp = KP_DEFAULT;
	Ki = KI_DEFAULT;
	Kd = KD_DEFAULT;
	
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	// Set the errors to a known values.
	p_error = PID::INITIAL_ERROR;
	i_error = PID::INITIAL_ERROR;
	d_error = PID::INITIAL_ERROR;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error; // The differentciation part is a substraction.
	p_error = cte;
	i_error += cte; // The integral part is a sum.
}

double PID::TotalError() {
	double alpha = Kp * p_error + Kd * d_error + Ki * i_error;
	return alpha;
}

