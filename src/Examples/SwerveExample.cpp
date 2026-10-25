/**
 * This is an example file for using the Swerve kinematics class
 *
 * */

#include "SwerveImpl.hpp"
#include "VexLib/Localizer.hpp"
#include "VexLib/SwerveKinematics.hpp"
#include "vex.h"
#include "vex_controller.h"
#include "vex_global.h"
#include "vex_thread.h"
#include <vector>

#ifdef SWERVE_TEST

#define swerveVexMain main

#endif

using VexLib::Pose2D;

vex::brain Brain;
float VOLTAGE; // used by SwerveImpl to ajust power based on battery voltage


int swerveVexMain(){

	VOLTAGE = Brain.Battery.voltage();
	std::vector<Pose2D> PodPositions(3); // note that we can use any number of pods (even 1 pod will work)
	
	PodPositions[0] = Pose2D(120, 0);
	PodPositions[1] = PodPositions[2] = PodPositions[0];
	PodPositions[1].rotateBy(VexLib::deg, 120);
	PodPositions[2].rotateBy(VexLib::deg, 60);
	// we assumed that the wheels are arranged in an equilateral triangle

	DiffyPod pods[] = {
		DiffyPod(vex::PORT1, vex::PORT2),
		DiffyPod(vex::PORT3, vex::PORT4),
		DiffyPod(vex::PORT5, vex::PORT6)
	}; // DiffyPod is a helper class that moves the pods to their coresponding positions and velocities given by the Swerve kinematics
	
	VexLib::Swerve kinematics(
		PodPositions,
		{
		// see the Swerve kinematics class for details on how to implement the helper functions
		[&](const Pose2D& p) { pods[0].setVect(p); },
		[&](const Pose2D& p) { pods[1].setVect(p); },
		[&](const Pose2D& p) { pods[2].setVect(p); }
		}
	);

	// update each pod orientation (we used only motor encoders)
	vex::thread t1([](void* i){
		DiffyPod *pods = static_cast<DiffyPod*>(i);
		while(1){
			pods[0].update();
			pods[1].update();
			pods[2].update();
			wait(0.1, vex::msec);
		}
	}, pods); // very very optional, might get removed if using rotary encoders
	
	vex::controller c(vex::primary);
	while(1){
		Brain.Screen.clearScreen();
	}

	return 0;
}

#ifdef SWERVE_TEST
#undef swerveVexMain
#endif
