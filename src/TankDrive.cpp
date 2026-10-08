#include "vex.h"
#include "TankDrive.h"
#include <algorithm>


extern vex::brain Brain;

void TankDrive::drive(float fwd, float rot){

    float BATTERY_VOLTAGE = Brain.Battery.voltage(vex::volt);

    float lpow = fwd + rot;
    float rpow = fwd - rot;
    float div  = std::max(fabs(lpow), fabs(rpow));

    lpow /= div;
    rpow /= div;

    left.spin(vex::directionType::fwd, lpow * BATTERY_VOLTAGE, vex::voltageUnits::volt);
    left.spin(vex::directionType::fwd, rpow * BATTERY_VOLTAGE, vex::voltageUnits::volt);

}

void TankDrive::lockRotation(VexLib::AngleUnits u, float angle, bool lock){
    if(!lock){
        setLockRotation(false);
        return;
    }
    setLockRotation(true);
    
}
