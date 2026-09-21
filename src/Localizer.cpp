#include "Localizer.hpp"
#include "Units.hpp"
#include "VexLib.hpp"
#include <cmath>

namespace VexLib{

double Pose2D::getX(VexLib::DistanceUnits d) const {
	return VexLib::convertDistance(distUnit, d, x);
}

double Pose2D::getY(VexLib::DistanceUnits d) const {
	return VexLib::convertDistance(distUnit, d, y);
}

double Pose2D::getH(VexLib::AngleUnits a) const {
	return VexLib::convertAngles(angleUnits, a, h);
}
double getDistance(const Pose2D& a, const Pose2D& b){
	return std::hypot(a.x - b.getX(a.distUnit), a.y - b.getY(a.distUnit));
}
void Pose2D::rotateBy(VexLib::AngleUnits a, double angle){
	using std::sin;
	using std::cos;
	angle = VexLib::convertAngles(a, VexLib::AngleUnits::rad, angle);
	double sx = this->x;
	double sy = this->y;
	x = sx * cos(angle) - sy * sin(angle);
	y = sx * sin(angle) + sy * cos(angle);
}

Pose2D operator -(const Pose2D& a, const Pose2D& b){
	return Pose2D (
		a.x - convertDistance(b.distUnit, a.distUnit, b.x),
		a.y - convertDistance(b.distUnit, a.distUnit, b.y),
		a.h,
		a.distUnit,
		a.angleUnits
	);
}

Pose2D operator +(const Pose2D& a, const Pose2D& b){
	return Pose2D(
		a.x + convertDistance(b.distUnit, a.distUnit, b.x),
		a.y + convertDistance(b.distUnit, a.distUnit, b.y),
		a.h,
		a.distUnit,
		a.angleUnits
	);
}

Pose2D operator *(const Pose2D& a, double f){
	return Pose2D(
		a.x * f,
		a.y * f,
		a.h,
		a.distUnit,
		a.angleUnits
	);
}
Pose2D operator /(const Pose2D& a, double f){
	return Pose2D(
		a.x / f,
		a.y / f,
		a.h,
		a.distUnit,
		a.angleUnits
	);
}

Pose2D Localizer::getEstimatedVelocity(DistanceUnits d, TimeUnits t, AngleUnits a){
	return (getEstimatedPose() - lastPose) / getElapsedTime(TimeUnits::s);
}

void Localizer::setPosition(const Pose2D& pose){
	lastPose = pose;
}

void Localizer::resetTracking(){
	this->setPosition(Pose2D());
}

}; // namespace VexLib
