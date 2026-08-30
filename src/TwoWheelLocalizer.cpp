#include "Localizer.hpp"
#include "TwoWheelLocalizer.hpp"
#include "Units.hpp"
#include <cmath>

namespace VexLib{
	
TwoWheelLocalizer::TwoWheelLocalizer(const Pose2D& paralelWheelPosition, 
					  const Pose2D& perpendicularWheelPosition, 
					  std::function<double(AngleUnits)> getangle,
					  std::function<double(DistanceUnits)> getdX,
					  std::function<double(DistanceUnits)> getdY):
		parallelWheelPos{paralelWheelPosition},
		perpendicularWheelPos{perpendicularWheelPosition},
		getAngle{std::move(getangle)},
		getDX{std::move(getdX)},
		getDY{std::move(getdY)}{}


Pose2D TwoWheelLocalizer::getEstimatedPose(DistanceUnits d, AngleUnits a){
	return Pose2D(
		convertDistance(lastPose.distUnit, d, lastPose.x),
		convertDistance(lastPose.distUnit, d, lastPose.y),
		convertAngles(lastPose.angleUnits, a, lastPose.h),
		d,
		a
	);
}

#define CDU currentPos.distUnit
#define CAU currentPos.angleUnits

// note: maybe in the future we will not use std's implementation for sine and cosine in order to get faster results
// TODO: add constant acceleration estimation (i hope not)
void TwoWheelLocalizer::update(){
	using std::sin;
	using std::cos;
	Pose2D currentPos = this->getEstimatedPose();
	double heading = getAngle(VexLib::AngleUnits::rad);
	double dH = heading - currentPos.h;
	double strf = getDY(CDU) - parallelWheelPos.getX(CDU) * dH;

	double fwd = getDX(CDU) - perpendicularWheelPos.getX(CDU) * dH;
	Pose2D dpose;
			

	if(fabs(heading) <= VexLib::convertAngles(VexLib::AngleUnits::deg, CAU, 0.3)){
		// linear estimation for near linear movement
		dpose = Pose2D(
				fwd * cos(heading) - strf * sin(heading),
				fwd * sin(heading) + strf * cos(heading),
				dH,
				CDU,
				VexLib::AngleUnits::rad
		);
	} else {
		// 2nd order estimation (asume constant velocity on both movement and spin)
		
		double cosIntegral = sin(heading) - sin(currentPos.getH(VexLib::AngleUnits::rad));
		double sinIntegral = cos(currentPos.getH(VexLib::AngleUnits::rad)) - cos(heading);

		dpose = Pose2D(
				fwd / dH * cosIntegral - strf / dH * sinIntegral,
				fwd / dH * sinIntegral + strf / dH * cosIntegral,
				dH,
				CDU,
				VexLib::AngleUnits::rad
		);
	}
	setPosition(currentPos + dpose);
}

#undef CUD
#undef CAU


}; // namespace VexLib
