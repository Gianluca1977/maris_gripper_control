#ifndef CONTROLLERDATA_H
#define CONTROLLERDATA_H

#include "wf.h"

#define MAX_CONF	6
#define NUM_MOT     3

class ControllerData
{
public:

    ControllerData();

    static WF::Semaphore MsgSem;

    static int nodeIds[NUM_MOT];

    static bool armPresent;

    static struct sockTCP_t
    {
        bool active;
        int port;
    } sockTCP;

    static struct homeConfig_t
    {
        int safe_curr_limit;
        int peak_curr;
        int cont_curr;
        int max_speed;
    } homeConfig;

    static struct operConfig_t
    {
        int safe_curr_limit;
        int peak_curr;
        int cont_curr;
        int max_speed;
        int max_acc;
        int max_deacc;
    } operConfig;

    static int finger_conf_num;
    static double finger_confs[MAX_CONF][NUM_MOT];

    static bool TcpActive;

    /*
    unsigned int DOF_;
    unsigned int nfingers;
    bool isHomeDone;
    */

    static bool doHome;
    //int auxInternalCall;
    static bool fault;
    static bool tcpDoHome;
    static bool tcpIsHomeDone;
    //bool setTactOffset;

    /********   RosInterface Stuffs  ********/
#ifdef ROS_IF

#endif
    // --- input variables from the RosInterface  --- //
    /*
     * int srv_preshape;
    double srv_radius;
    double	srv_radius_palm;
    float action_force;
    int action_goal;
    */
    static bool emerg_stop;

    static int srv_mode;

    // --- Output variables to the RosInterface  --- //
    /*
    double action_perc_complete;
    bool action_done;
    bool action_free;

    int fingerOffset;
    */
    /*****************************************/
};

#endif // CONTROLLERDATA_H
