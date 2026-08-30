#ifndef Localizer_HPP
#define Localizer_HPP

#include "Units.hpp"
namespace VexLib{

typedef struct Pose2D {
	double x, y, h;
	DistanceUnits distUnit = DistanceUnits::mm;
	AngleUnits angleUnits = AngleUnits::rad;
	Pose2D(double x = 0.0, 
           double y = 0.0, 
           double h = 0.0, 
           DistanceUnits distUnit = DistanceUnits::mm, 
           AngleUnits angleUnits = AngleUnits::rad)
        : x(x), y(y), h(h), distUnit(distUnit), angleUnits(angleUnits) {}
	double getX(VexLib::DistanceUnits);
	double getY(VexLib::DistanceUnits);
	double getH(VexLib::AngleUnits);
} Pose2D ;

Pose2D operator -(const Pose2D&, const Pose2D&);
Pose2D operator +(const Pose2D&, const Pose2D&);
Pose2D operator *(const Pose2D&, double);
Pose2D operator /(const Pose2D&, double);


class Localizer {
protected:
	Pose2D lastPose;
public:

	Localizer() = default;
	virtual Pose2D getEstimatedPose(DistanceUnits = mm, AngleUnits = rad) = 0; // pure virtual, we want the user to define its own localizer
	virtual Pose2D getEstimatedVelocity(DistanceUnits = mm, TimeUnits = s, AngleUnits = rad);
	virtual void update(){}
	
	/* 
	 * @breif sets the estimated position to (x=0, y=0, h=0) and restarts tracking
	 * 
	 * */
	void resetTracking();
	
	/*
	 * @brief updates the current position to pose
	 * */
	void setPosition(const Pose2D& pose);
	virtual void operator ()() = 0;

};

}; // namespace VexLib

#endif
