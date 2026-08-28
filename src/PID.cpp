#include "PID.hpp"
#include "Units.hpp"
#include "VexLib.hpp"

VexLib::PIDFController::PIDFController(VexLib::PIDFCoefficients pidfc):
		coeff{pidfc}, Isum{0}, minSO{0}, maxSO{0}, frequency{50}{}

void VexLib::PIDFController::setControllerFrequency(double f){ frequency = f; }

void VexLib::PIDFController::setMaxSystemOutput(double min, double max){
	minSO = min;
	maxSO = max;
}

void VexLib::PIDFController::resetTracking(){ Isum = 0; }

double VexLib::PIDFController::getPowerOutputWithVelo(double error, double velocity){
	if(lastTime < 0) lastTime = getElapsedTime(TimeUnits::s);
	if(getElapsedTime(TimeUnits::s) - lastTime < 1.0 / frequency)
		return lastReturn;

	double P = error * coeff.P;
	Isum += error * coeff.I;
	double D = velocity * coeff.D;

	double ret = P + D + Isum + coeff.F;
	if((ret > maxSO || ret < minSO) &&  // saturated controller
		error * ret > 0 				// integral is pushing the system past its limits
		){
		ret -= Isum;
	}
	lastReturn = ret;
	lastTime = getElapsedTime(TimeUnits::s);

	return ret;

}

double VexLib::PIDFController::getPowerOutput(double error){
	double deltaTime = getElapsedTime(TimeUnits::s) - lastTime;
	double r = getPowerOutputWithVelo(error, (error - lastError) / deltaTime);
	lastError = error;
	return r;
}

void VexLib::PIDFController::setPIDFCoefficinet(PIDFCoefficients c){ coeff = c; }


