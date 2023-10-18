#include "Arduino.h"
#include "PID.h"

PID::PID(double kp, double ki, double kd){ // double input, double output, double setpoint
	// Setup initial PID variables
	//_input = input;
	//_output = output;
	//_setpoint = setpoint;
	_kp = kp;
	_ki = ki;
	_kd = kd;
	_tBefore = millis(); // make sure that _tBefore is initialised with a value before calling calculatePID
	_errorSum = 0;
	_errorBefore = 0;
	
}


double PID::calculatePID(double input, double setpoint){
	// Find time change since last function call
	_tNow = millis();
	_tChange = _tNow - _tBefore;
	
	// Calcuate error
	_error = setpoint - input;
	_errorSum += _error * _tChange; //Integration estimation. approximate dt as change in time
	_errorChange = (_error - _errorBefore) / _tChange; // Differentiation estimation. approximate dt as change in time
	// Calculate PID Output
	_P = _error * _kp; // Proportional term, multiply error directly by its constant
	_I = _errorSum * _ ki; // Integral term, muliply integration estimate by its constant
	_D = _errorChange * _kd // Differential term, multiply differentiation estimate by its constant
	
	return (_P + _I + _D);
}

void PID::setConstants(double kp, double ki, double kd){
	_kp = kp;
	_ki = ki;
	_kd = kd;
}