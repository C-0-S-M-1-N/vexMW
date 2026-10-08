#include "Localizer.hpp"
#include "Paths.hpp"
#include "TwoWheelLocalizer.hpp"
#include "Units.hpp"
#include "Display.hpp"
#include "vex.h"
#include "VexLib.hpp"
#include "vex_brain.h"
#include "vex_color.h"
#include "vex_global.h"
#include "vex_thread.h"
#include "vex_triport.h"

#include "vex_motor.h"
#include "TankDrive.h"
#include "vex_motorgroup.h"

#include "vex_units.h"
#include <memory>
#include <vector>

#define PERPENDICULAR_ENCODER_PORT A
#define PARALLEL_ENCODER_PORT C
#define Encoder_Wheel_Radius 32

vex::brain Brain;
using VexLib::Pose2D;
vex::controller Controller = vex::controller();

const float BATTERY_VOLTAGE = Brain.Battery.voltage(vex::volt);

VexLib::Localizer *localizer = nullptr;
vex::gyro imu(Brain.ThreeWirePort.E);

double getElapsedTime(VexLib::TimeUnits unit){
    uint64_t timeuS = Brain.Timer.systemHighResolution();
    return VexLib::convertTime(VexLib::TimeUnits::us, unit, timeuS);
}

// localizer init example
void initLocalizer(){
	std::shared_ptr<vex::encoder> 
		parallelEnc = std::make_shared<vex::encoder>(Brain.ThreeWirePort.PARALLEL_ENCODER_PORT),
		perpEnc = std::make_shared<vex::encoder>(Brain.ThreeWirePort.PERPENDICULAR_ENCODER_PORT);

	localizer = new VexLib::TwoWheelLocalizer(
		Pose2D(/*TODO: add value*/), Pose2D(/*TODO: add value*/),
	[](VexLib::AngleUnits cau){
		return VexLib::convertAngles(VexLib::AngleUnits::deg, cau, imu.heading(vex::deg));
	},
	[parallelEnc](VexLib::DistanceUnits cdu){
		static int16_t lastReading = 0;
		int16_t reading = parallelEnc->position(vex::rotationUnits::rev);
		double ret = (reading - lastReading) * Encoder_Wheel_Radius;
		lastReading = reading;
		return ret;
	},
	[perpEnc](VexLib::DistanceUnits cdu) -> double {
		static int16_t lastReading = 0;
		int16_t reading = perpEnc->position(vex::rotationUnits::rev);
		double ret = (reading - lastReading) * Encoder_Wheel_Radius;
		lastReading = reading;
		return ret;
	}
);

}

int randare(void *d){
	Display *disp = (Display*)(d);
	while(1){
		disp->renderScreen();
		vex::this_thread::sleep_for(34);
	}
	return 0;
}

#define accuracy 0.001


int main(){
	Display display = Display();
	display.onUserCreate(); ///seteaza doar offset-urile momentan
//	initLocalizer();
//	vex::thread localizerUpdate([]() -> void { localizer->update(); });
	std::vector<Pose2D> p({Pose2D(100, 150), Pose2D(320, 150), Pose2D(170, 180)});
	VexLib::BeziereCurve traj(p);
	std::function<Pose2D(double)> tF = (traj.getPathFunction());

	vex::thread tred(randare, &display);
    while(1){
		display.onUserUpdate(getElapsedTime(VexLib::TimeUnits::s), Controller);

		if(display.tastaturaAConfirmat()){
			printf("%s\n\n", display.getTastaturaText().c_str());
		}

		vex::task::sleep(20);
    }
	if(localizer != nullptr)
		delete localizer;
}
