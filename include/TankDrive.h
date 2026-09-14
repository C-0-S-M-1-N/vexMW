#ifndef TankDrive_h
#define TankDrive_h
#include "Paths.hpp"
#include "vex.h"
#include "PID.hpp"
#include "Units.hpp"
#include "Localizer.hpp"

extern VexLib::Localizer* localizer;

class TankDrive {
    /* Autonomous parameters */
    VexLib::PIDFController HeadingController;
	VexLib::PIDFController TranslationalController;

	VexLib::Path* currentPath;
	VexLib::Localizer* localizer;

    /* Driver controlled params*/

#ifndef TEST
    vex::motor_group left, right; 
#endif
    float angleLock = 0;
    bool FLAGS = 0;

    inline bool isRotationBlocked() { return FLAGS & 0b1; }
    inline void setLockRotation(bool lock) { lock ? FLAGS |= 0x1 : FLAGS &= 0xFE; }

    inline bool isRunningPath() { return FLAGS & 0b10; }
    inline void setRunPath(bool set) { set ? FLAGS |= 0x02 : FLAGS &= 0xFD; }

public:
#ifndef TEST
    TankDrive(const vex::motor_group& LEFT_SIDE_MOTORS, const vex::motor_group& RIGHT_SIDE_MOTORS, VexLib::Localizer* Localizer = nullptr):
	localizer {Localizer},
    left{LEFT_SIDE_MOTORS}, right{RIGHT_SIDE_MOTORS}{}
#endif
	
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
