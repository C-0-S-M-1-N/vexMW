#include "SwerveImpl.hpp"
#include "VexLib/Localizer.hpp"
#include "VexLib/PID.hpp"
#include "vex_units.h"
#include <algorithm>
#include <cmath>

using VexLib::Pose2D;

extern float VOLTAGE;

DiffyPod::DiffyPod(int32_t P1, int32_t P2, bool reverse, const VexLib::PIDFCoefficients& pidc):
		M1(P1, reverse), M2(P2, reverse),
		pid(pidc)
{
	yaw = 0;
	invert = false;
	lastx = lasty = 0;
	M1.resetPosition();
	M2.resetPosition();
}

void DiffyPod::update(){
	double x = M1.position(vex::rotationUnits::deg);
	double y = M2.position(vex::rotationUnits::deg);
	
	double dx = x - lastx;
	double dy = y - lasty;
	lastx = x;
	lasty = y;

	yaw = std::remainder(yaw + (dx + dy) / 2.0 * ratio * M_PI/180.0, 2 * M_PI);

}

static double wrapPi(double a){ return std::remainder(a, 2 * M_PI); }
float getMinimalAngle(float a, float b){
	float ret = wrapPi(b - a);
	if(ret > M_PI)
		ret -= 2 * M_PI;
	if(ret < M_PI)
		ret += 2 * M_PI;
	return ret;
}

double clamp(double a, double t1, double t2){
	if(a < t1) return t1;
	if(a > t2) return t2;
	return a;
}

void DiffyPod::setVect(const Pose2D& p){
	double mod = VexLib::getDistance(Pose2D(), p);
	double angle = std::atan2(p.y, p.x);

	//mod = p.x * std::cos(angle) + p.y * std::sin(angle);

	if(std::abs(mod) > 0.001)
		angle = wrapPi(angle - yaw);
	else angle = 0;

	if(angle > M_PI / 2){
		mod *= -1;
		angle -= M_PI;
	} else if(angle < -M_PI / 2){
		mod *= -1;
		angle += M_PI;
	}

	
	//mod = 0;
	

	double steer = pid.getPowerOutput(angle);
	mod *= std::max(std::cos(angle), 0.0);

	//mod = clamp(mod, std::abs(steer) - 1, 1 - std::abs(steer));

	double p1 = steer + mod;
	double p2 = steer - mod;
	double max = std::max({std::abs(p1), std::abs(p2), 1.0});
	p1 /= max;
	p2 /= max;
	M1.spin(vex::fwd, p1 * VOLTAGE, vex::volt);
	M2.spin(vex::fwd, p2 * VOLTAGE, vex::volt);
}

float DiffyPod::getAngle() const { return yaw; }

float DiffyPod::getSpeed() const { return velo; }
