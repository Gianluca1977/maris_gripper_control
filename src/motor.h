#ifndef MOTOR_H
#define MOTOR_H

#include "can_driver.h"
#include "supervisor.h"

#define INIT_WAIT           -1
#define INIT_BOOTUP_DEV		0
#define INIT_START_DEV		1
#define INIT_SHUTDOWN_DEV	2
#define INIT_SWITCH_ON_DEV	3
#define INIT_ENABLE_OP_DEV	4
#define INIT_ENABLED		5
#define INIT_TRACE_CONF		6
#define INIT_SYNC_CONF		7
#define INIT_TPDO1_CONF		8
#define INIT_FAULHABER_CONF	9
#define INIT_DONE       	10
#define INIT_FAULT          11
#define INIT_CLOSE          12
#define INIT_NONE           13

class Motor : virtual public CanInterface
{
public:
    //MotorData Data[NUM_MOT];
    int ID;

    int MaxPeakCurr;
    int MaxContCurr;
    int MaxVel;
    int MaxAcc;
    int MaxDeacc;

    union {
        int State;
        char State_byte[2];
    };

    union {
        int Old_State;
        char Old_State_byte[2];
    };

    long Velocity;	//last found vel
    long Position;	//last found pos
    long Current;
    long Control;
    double PositionGrad;
    double MaxPosGrad;
    double MinPosGrad;

    bool Operational;
    bool TargetReached;
    bool BootUp;
    bool Fault;

    Motor();
    Motor(int id);
    ~Motor();

    //MotorData &operator[](int idx) { return Data[idx]; }

    void setID(int id);
    void setLimits(long maxVel = 500, long maxAcc = 1, long maxDeacc = 1, long maxPCurr = 3000, long maxCCurr = 1000);
    void stop();
    void emergencyStop();
    bool checkStopped();
    void enable();
    void disable();
    void reset();
    void clear();
    void init(int phase);

    void stateUpdate(unsigned char data[]);

    bool stateChanged(){return ((State & ~TARGET_MASK) != (Old_State & ~TARGET_MASK));}

    void setHomePosition();
    void setMaxAcc(long maxAcc = 1);
    void setMaxDeacc(long maxDeacc = 1);
    void setMaxPeakCurr(long maxPCurr = 3000);
    void setMaxContCurr(long maxCCurr = 1000);
    void setMaxVel(long maxVel = 500);
    void queryAsyncVel();
    void moveVel(long vel = 0);
    void movePosAbs(long absValue = 0);
    void loadPosAbs(long absValue = 0);
    void startMovePos();
    void movePosRel(long relValue= 0);
};

#endif // MOTOR_H
