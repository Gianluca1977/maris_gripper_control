#ifndef GRIPPER_H
#define GRIPPER_H

#include "motorguard.h"
#include "controllerdata.h"

#define MOTOR_GUARD

class Gripper : virtual public ControllerData
{
public:

#ifdef MOTOR_GUARD
    static MotorGuard Motors[NUM_MOT];
#else
    static Motor Motors[NUM_MOT];
#endif

    //Gripper();
    Gripper(int []);

    void Open(int vel = 0);
    void Close(int vel = 0);

    void disableMotors();
    void enableMotors();

    void update();
    void emergencyStop();

    void stop();
    bool checkStopped();

    void movePosAbsGrad(int64_t req_pos[NUM_MOT]);
    void movePosAbsGrad(double req_pos[NUM_MOT]);
    void moveVel(int64_t req_vel[NUM_MOT]);
    void goFinalPos(bool motor_selection[NUM_MOT]);
    void setHomePos(bool motor_selection[NUM_MOT]);
    void setHomePos(void);
    void setFinalPos(bool motor_selection[NUM_MOT]);

    bool isOperative();

    bool commandExecuted();

    bool hasID(int nodeID);
    int getMotorIndex(int nodeID);

    void updateStates(TPCANMsg msg);

    double pcanData2Double(TPCANMsg msg, int offset);

protected:
    void saveMaxLimit(int nodeId = -1, long pos = 0);
    double roundToSignificant(double num, int significant);
};

#endif // GRIPPER_H
