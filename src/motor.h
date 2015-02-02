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
/*
struct MotorData
{
    int ID;

    int MaxHomePeak;
    int MaxHomeCont;
    int MaxVel;
    int MaxAcc;
    int MaxDeacc;

    int State;
    int Old_State;

    long Velocity;	//last found vel
    long Position;	//last found pos
    long Current;
    long Control;
    double PositionGrad;
    double MaxPosGrad;
    double MinPosGrad;

    bool Operational;
    bool PosReached;
    bool BootUp;
    bool Fault;
};
*/
class Motor : virtual public CanInterface
{
public:
    //MotorData Data[NUM_MOT];
    int ID;

    int MaxHomePeak;
    int MaxHomeCont;
    int MaxVel;
    int MaxAcc;
    int MaxDeacc;

    int State;
    int Old_State;

    long Velocity;	//last found vel
    long Position;	//last found pos
    long Current;
    long Control;
    double PositionGrad;
    double MaxPosGrad;
    double MinPosGrad;

    bool Operational;
    bool PosReached;
    bool BootUp;
    bool Fault;

    Motor();
    Motor(int id);
    ~Motor();

    //MotorData &operator[](int idx) { return Data[idx]; }

    void setID(int id);
    void stop();
    bool checkStopped();
    void enable();
    void disable();
    void reset();
    void init(int phase);

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
