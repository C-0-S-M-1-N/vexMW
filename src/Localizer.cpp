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

Pose2D operator -(const Pose2D& a, const Pose2D& b){
	return Pose2D (
		a.x - convertDistance(b.distUnit, a.distUnit, b.x),
		a.y - convertDistance(b.distUnit, a.distUnit, b.y),
		a.h - convertAngles(b.angleUnits, a.angleUnits, b.h),
		a.distUnit,
		a.angleUnits
	);
}

Pose2D operator +(const Pose2D& a, const Pose2D& b){
	return Pose2D(
		a.x + convertDistance(b.distUnit, a.distUnit, b.x),
		a.y + convertDistance(b.distUnit, a.distUnit, b.y),
		a.h + convertAngles(b.angleUnits, a.angleUnits, b.h),
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
