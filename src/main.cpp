#include "Localizer.hpp"
#include "TwoWheelLocalizer.hpp"
#include "Units.hpp"
#include "vex.h"
#include "VexLib.hpp"
#include "vex_global.h"
#include "vex_thread.h"
#include "vex_triport.h"
#include "vex_units.h"
#include <memory>

#define PERPENDICULAR_ENCODER_PORT A
#define PARALLEL_ENCODER_PORT C
#define Encoder_Wheel_Radius 32

vex::brain Brain;
using VexLib::Pose2D;

const float BATTERY_VOLTAGE = Brain.Battery.voltage(vex::volt);

VexLib::Localizer *localizer;
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
		Pose2D(), Pose2D(),
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
	[perpEnc](VexLib::DistanceUnits cdu){
		static int16_t lastReading = 0;
		int16_t reading = perpEnc->position(vex::rotationUnits::rev);
		double ret = (reading - lastReading) * Encoder_Wheel_Radius;
		lastReading = reading;
		return ret;
	}
);

}

int main(){
	initLocalizer();
	vex::thread localizerUpdate([]() -> void { localizer->update(); });

    while(1){

    }
	delete localizer;
}
