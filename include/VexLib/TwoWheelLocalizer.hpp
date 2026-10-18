#ifndef TwoWhellLocalizer_hpp
#define TwoWhellLocalizer_hpp

#include "Localizer.hpp"
#include "Units.hpp"
#include <functional>

namespace VexLib{

class TwoWheelLocalizer: public Localizer {
	Pose2D parallelWheelPos, perpendicularWheelPos;
	std::function<double(AngleUnits)> getAngle;	
	std::function<double(DistanceUnits)> getDX;
	std::function<double(DistanceUnits)> getDY;

public:
	/* 
	 * @brief constructor for two wheel localizer (note: use only the x coord for measurements)
	 * @param paralelWheelPosition position of the paralel wheel based on the center of rotation
	 * @param perpendicularWheelPosition position of the perpendicular wheel based on the center of rotation
	 * @param getAngle function that returns the angle of the robot
	 * @param getDX function that returns the X distance traveled since last call
	 * @param getDY function that returns the Y distance traveled since last call
	 * */
	TwoWheelLocalizer(const Pose2D& paralelWheelDistFromCenter, 
					  const Pose2D& perpendicularWheelDistFromCenter, 
					  std::function<double(AngleUnits)> getAngle,
					  std::function<double(DistanceUnits)> getDX,
					  std::function<double(DistanceUnits)> getDY);

	Pose2D getEstimatedPose(DistanceUnits = mm, AngleUnits = rad);
	
	/* @brief use this method to update the estimate position
	 * */
	void update();
	void operator ()(){ update(); }
};

}; // namespace VexLib


#endif
