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
 
/*
 * Implementation details:
 *
 * * * * * linear estimation: * * * * *
 *
 * start from adding to the current position the velocity that the robot is heading:
 *
 * x += dV_x * dt (= dx);
 * y += dV_y * dt (= dy);
 *
 * now we need to compute the velocities
 *
 * dV_x = dx / dt = velocity from the forward dead wheel encoder rotated at the current robot angle (so that we change the reference frame from robot-centric to field-centric)
 *
 * dx / dt = dFwd / dt * cos(heading) - dLat / dt * sin(heading)
 * (analog for dy)
 * dy / dt = dFwd / dt * sin(heading) + dLat / dt * sin(heading)
 * 
 * (matrix form)
 * (dx / dt) = (dFwd / dt) (cos(heading)  -sin(heading))
 * (dy / dt)   (dLat / dt) (sin(heading)   cos(heading))
 *
 * now just multiply everything by dt and we get the linear aproximation dx, dy
 *
 *
 * * * * * constant velocity arc based estimation: * * * * *
 *
 * assume that the robot spins and moves with constant velocities (and for small periods of time (ie ~0.5ms) it aproximates quite good!)
 * constinueing from where we left on linear estimation without multiplying bt dt (since we dont know dt as it can be any arbitrary value)
 *
 * (note: dVar is the change of Var in one loop-time)
 *
 * (dx / dt) (t) = (dFwd / dt) (cos(dH/dt * t + heading)  -sin(dH/dt * t + heading))
 * (dy / dt)       (dLat / dt) (sin(dH/dt * t + heading)   cos(dH/dt * t + heading))
 *
 * to get dx and dy we will integrate from 0 to T in order to get the current distances that the bot made since last looptime (remember that everything is in reference to the last frame)
 *
 * dx(dt) = dFwd / dt * dt / dH * (sin(dH/dt * t + heading) - cos(dH / dt * t + heading)) from 0 to dt
 * dx(dt) = dFwd / dH * (sin(dH/dt * dt + heading) - sin(heading) - cos(dH / dt * dt + heading) + cos(heading))
 * analog for dy
 *
 * * * * * higher order estimations * * * * *
 *
 * as we assumed constant velocity we can also assume constant acceleration, jerk, snap, crackle, pop, ...
 * the ideea is the same
 *
 * Fwd(t) = pop * t^6 + crackle*t^5 + snap * t^4 + jerk * t^3 + acceleration * t^2 + velocity * t + 0
 * Lat(t) = pop * t^6 + crackle*t^5 + snap * t^4 + jerk * t^3 + acceleration * t^2 + velocity * t + 0
 * H(t) = pop * t^6 + crackle*t^5 + snap * t^4 + jerk * t^3 + acceleration * t^2 + velocity * t + 0
 *
 * dx(t) / dt = dFwd(t) / dt * cos(H(t)) + dLat(t) / dt * sin(H(t))
 * aproximate dx(dt) (since we can't solve the integral for any constant other than velocity) with simpson rule
 * but we also need to explicitly find all constants (pop, crackle, snap, jerk, acceleration, velocity), to do this we 
 * can use the LIP (Lagrange Interpolation polinom) by remembering the last n values (dFwd, dLat, dH, dt) and doing partial sums to get the "true" points
 * in order to interpolate.
 * 
 *
 * */
