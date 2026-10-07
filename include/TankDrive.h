#ifndef TankDrive_h
#define TankDrive_h
#include "Paths.hpp"
#include "vex.h"
#include "PID.hpp"
#include "Units.hpp"
#include "Localizer.hpp"
#include "vex_motorgroup.h"


class TankDrive {
    /* Autonomous parameters */
    VexLib::PIDFController HeadingController = VexLib::PIDFController({1.2, 0, 0, 0});
	VexLib::PIDFController TranslationalController = VexLib::PIDFController({0, 0, 0, 0});

	VexLib::Path* currentPath;
	VexLib::Localizer* localizer;
	double pullRadius = 300;

    /* Driver controlled params*/

    vex::motor_group left, right; 
    bool FLAGS = 0;

    inline bool isRotationLocked() { return FLAGS & 0b1; }
    inline void setLockRotation(bool lock) { lock ? FLAGS |= 0x1 : FLAGS &= 0xFE; }

    inline bool isRunningPath() { return FLAGS & 0b10; }
    inline void setRunPath(bool set) { set ? FLAGS |= 0x02 : FLAGS &= 0xFD; }

	double getClosestPointToPath(double step = 0.01, int iter = 30);
	VexLib::Pose2D PathFollowerVector();

public:
    TankDrive(const vex::motor_group& LEFT_SIDE_MOTORS, const vex::motor_group& RIGHT_SIDE_MOTORS, VexLib::Localizer* Localizer = nullptr):
	localizer {Localizer},
    left{LEFT_SIDE_MOTORS}, right{RIGHT_SIDE_MOTORS}{}
	
	/* Driver controlled functions */
    void drive(float forward, float rotate);
    void lockRotation(VexLib::AngleUnits, float, bool lock = true);
 
	/* Autonomous controll functions */

	// Use this to update the autonomous drivetrain to follow the path correctly
	void updateAutonomousDrive();

	// Set the localizer that the drivetrain will follow
	void setLocalizer(VexLib::Localizer*);

	// Set the path that the drivetrain will follow
	void setFollowingPath(VexLib::Path*);

};

#endif
