#include "Localizer.hpp"
#include "TwoWheelLocalizer.hpp"
#include "Units.hpp"
#include <cmath>

namespace VexLib{
	
TwoWheelLocalizer::TwoWheelLocalizer(const Pose2D& paralelWheelPosition, 
					  const Pose2D& perpendicularWheelPosition, 
					  std::function<double(AngleUnits)> getAngle,
					  std::function<double(DistanceUnits)> getDX,
					  std::function<double(DistanceUnits)> getDY):
		parallelWheelPos{paralelWheelPosition},
		perpendicularWheelPos{perpendicularWheelPosition},
		getAngle{getAngle},
		getDX{getDX},
		getDY{getDY}{}


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

void TwoWheelLocalizer::update(){
	Pose2D currentPos = this->getEstimatedPose();
	double dR = getAngle(CAU) - currentPos.h;
	Pose2D rotPos = Pose2D(
			getDX(CDU) * VexLib::cos(dR) - getDY(CDU) * VexLib::sin(dR),
			getDX(CDU) * VexLib::sin(dR) + getDY(CDU) * VexLib::cos(dR),
			dR,
			CDU,
			CAU
	);
	setPosition(currentPos + rotPos);
}

#undef CUD
#undef CAU


}; // namespace VexLib
