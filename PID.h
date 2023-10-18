#ifndef PID_H_
#define PID_H_

#include "Arduino.h"

class PID
{
	public:
		// Constructor
		PID(double kp, double ki, double kd); // double input, double output, double setpoint
		
		// Calculate PID output
		double calculatePID();
		void setConstants(double kp, double ki, double kd);
		
		
	private:
		// Time variables
		unsigned long _tBefore;
		unsigned long _tNow;
		unsigned long _tChange;
		
		// PID constants
		double _kp, _ki, _kd;
		
		// PID terms
		double _P, _I, _D;
		
		// Control variables
		// double _input, _output, _setpoint;
		
		// Error variables
		double _error, _errorSum, _errorBefore, _errorChange;
	
	
	
	
};


#endif