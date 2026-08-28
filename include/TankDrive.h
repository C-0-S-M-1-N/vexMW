#ifndef TankDrive_h
#define TankDrive_h
#include "vex.h"



namespace vex{

enum class angleUnits{
    deg,
    rad,
    NaA
};

}

class TankDrive{
    vex::motor_group left, right; 
    float angleLock = 0;
    bool FLAGS = 0;

    inline bool isRotationBlocked() { return FLAGS & 0b1; }
    inline void setLockRotation(bool lock) { lock ? FLAGS |= 0x1 : FLAGS &= 0xFE; }

    inline bool isRunningPath() { return FLAGS & 0b10; }
    inline void setRunPath(bool set) { set ? FLAGS |= 0x02 : FLAGS &= 0xFD; }

    

public:
    TankDrive(const vex::motor_group& LEFT_SIDE_MOTORS, const vex::motor_group& RIGHT_SIDE_MOTORS):
    left{LEFT_SIDE_MOTORS}, right{RIGHT_SIDE_MOTORS}{}

    void drive(float forward, float rotate);
    void lockRotation(vex::angleUnits, float);

};

#endif