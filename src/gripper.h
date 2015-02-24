#ifndef GRIPPER_H
#define GRIPPER_H

#include "motor.h"
#include "controllerdata.h"

class Gripper : virtual public ControllerData
{
public:

    static Motor Motors[NUM_MOT];

    //Gripper();
    Gripper(int []);

    void Open(int vel = 0);
    void Close(int vel = 0);

    void disableMotors();
    void enableMotors();

    void stop();
    bool checkStopped();

    bool isOperative();

    bool hasID(int nodeID);

    void updateStates(TPCANMsg msg);

    double pcanData2Double(TPCANMsg msg, int offset);

protected:
    void saveMaxLimit(int nodeId = -1, long pos = 0);
    double roundToSignificant(double num, int significant);

    static int jointReduction;
    static int safeJointOffset;
};

#endif // GRIPPER_H
