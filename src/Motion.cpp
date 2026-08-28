#include "Motion.hpp"
#include <cmath>
#include <algorithm>

namespace VexLib {

struct timeParam{
	double tA, tD, tC, MV;
};

timeParam getTimes(double A, double D, double& MV, double dist){
	MV = std::min(MV, std::sqrt(2 * dist * A * D / (A + D)));
	
	return { MV / A,  // tA
			 MV / D,  // tD
			 std::max(dist / MV - MV*(A+D)/(2*A*D), 0.0), //tC
			 MV};  	  // MV

}

double getTrapezoidalAcceleration(double Acceleration, double Decceleration, double MaxVelocity, double startPos, double endPos, double t){
	timeParam times = getTimes(Acceleration, Decceleration, MaxVelocity, std::abs(startPos - endPos));
	
	if(t < times.tA) return Acceleration;
	if(t - times.tA < times.tC) return 0;
	if(t - times.tA - times.tC < times.tD) return Decceleration;

	return 0;
}

double getTrapezoidalVelocity(double Acceleration, double Decceleration, double MaxVelocity, double startPos, double endPos, double t){
	timeParam times = getTimes(Acceleration, Decceleration, MaxVelocity, std::abs(startPos - endPos));


	if(t < times.tA) return Acceleration * t;
	if(t - times.tA < times.tC) return MaxVelocity;
	if(t - times.tA - times.tC < times.tD) return MaxVelocity - Decceleration * (t - times.tA - times.tC);

	return 0;
}


double getTrapezoidalPosition(double Acceleration, double Decceleration, double MaxVelocity, double startPos, double endPos, double t){
	timeParam times = getTimes(Acceleration, Decceleration, MaxVelocity, std::abs(startPos - endPos));
	
	signed sgn = startPos > endPos ? -1 : 1;
	
	double ret = 0;

	if(t < times.tA) return Acceleration / 2 * t * t * sgn;

	ret += Acceleration * times.tA * times.tA / 2 * sgn;

	if(t - times.tA < times.tC) return MaxVelocity * (t - times.tA) * sgn + ret;
	

	if(t - times.tA - times.tC < times.tD) 
			return ret + MaxVelocity * (t - times.tA) * sgn - Decceleration * sgn / 2.0 * (t - times.tA - times.tC) * (t - times.tA - times.tC);

	return endPos;
}

}; // namepsace VexLib
