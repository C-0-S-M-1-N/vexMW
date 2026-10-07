#include "Localizer.hpp"
#include "vex.h"
#include "TankDrive.h"
#include <algorithm>

extern const float BATTERY_VOLTAGE;
using VexLib::Pose2D;

void TankDrive::drive(float fwd, float rot){

    float lpow = fwd + rot;
    float rpow = fwd - rot;
    float div  = std::max(fabs(lpow), fabs(rpow));
	div = std::max(div, 1.f);

    lpow /= div;
    rpow /= div;
	
#ifndef TEST
    left.spin(vex::directionType::fwd, lpow * 12, vex::voltageUnits::volt);
    right.spin(vex::directionType::fwd, rpow * 12, vex::voltageUnits::volt);
#endif
}

void TankDrive::lockRotation(VexLib::AngleUnits u, float angle, bool lock){
    if(!lock){
        setLockRotation(false);
        return;
    }
    setLockRotation(true);
    
}

void TankDrive::setLocalizer(VexLib::Localizer* l){
	this->localizer = l;
}

void TankDrive::setFollowingPath(VexLib::Path* p){
	this->currentPath = p;
}

void TankDrive::updateAutonomousDrive(){
	Pose2D follow = PathFollowerVector();
	

}

Pose2D TankDrive::PathFollowerVector(){
	auto B = currentPath->getPathFunction();
	double t = getClosestPointToPath();
	
	Pose2D closePt = B(t);
	Pose2D d = currentPath->getDerivativePathFunction()(t);

	double len = VexLib::getDistance(Pose2D(), d);
	d = len > 1e-9 ? d * 1.0/len : Pose2D();

	Pose2D direction = closePt - localizer->getEstimatedPose();
	double dist = VexLib::getDistance(Pose2D(), direction);
	direction = direction * (dist > 1e-9 ? 1.0 / dist : 1);

	double weight = std::exp(-dist * dist / (pullRadius * pullRadius));
	double attraction = std::min(dist / pullRadius, 1.0);
	
	Pose2D help = localizer->getEstimatedPose() - closePt;
	if(t >= 1 - 1e-6 && 
		help.x * d.x + help.y * d.y > 0) weight = 0;

	return (d * weight) + (direction * attraction);
}

double TankDrive::getClosestPointToPath(double step, int iter){
	auto B = currentPath->getPathFunction();
	auto dist2 = [&](double t) -> double {
		Pose2D H = localizer->getEstimatedPose() - B(t);
		return H.x * H.x + H.y * H.y;
	};

	double bestT = 0, bestD2 = 1e300;
	for(double t = 0; t <= 1; t += step){
		double d2 = dist2(t);
		if(d2 < bestD2){
			bestD2 = d2;
			bestT = t;
		}
	}

	//refine search

		constexpr double g = 0.6180339887498949;
		double lo = std::max(0.0, bestT - step),
			   hi = std::min(1.0, bestT + step);
		
		double a = hi - g * (hi - lo), b = lo + g * (hi - lo);
		double fa = dist2(a), fb = dist2(b);
		for(int i = 0; i < iter; i ++){
			if(fa < fb){
				hi = b;
				b = a;
				fb = fa;
				a = hi - g * (hi - lo);
				fa = dist2(a);
			} else {
				lo = a;
				a = b;
				fa = fb;
				b = lo + g * (hi - lo);
				fb = dist2(b);
			}
		}
		return (lo + hi) / 2.0;
}
