#ifndef SwerveImpl_hpp
#define SwerveImpl_hpp

#include "VexLib/Localizer.hpp"
#include "VexLib/PID.hpp"
#include "vex.h"
#include <cstdint>

const double ratio = 32.0 / 107.0; 

extern float VOLTAGE;
class DiffyPod {
	float yaw;
	float lastyaw;
	bool invert;
	
	// maybe usefull someday... (not today)
	float yawDot;
	float velo;

	double lastx, lasty;

public:
	vex::motor M1, M2;
	VexLib::PIDFController pid;
	DiffyPod(int32_t P1, int32_t P2, bool reverse = false, const VexLib::PIDFCoefficients& pidc = {1.5, 0, 0.03, 0});
	
	void setVect(const VexLib::Pose2D& p);
	void update();
	float getAngle() const;
	float getSpeed() const;
};

#endif
