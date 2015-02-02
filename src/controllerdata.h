#ifndef CONTROLLERDATA_H
#define CONTROLLERDATA_H

#define MAX_CONF	6
#define NUM_MOT     3

class ControllerData
{
public:

    ControllerData();

    int nodeIds[NUM_MOT];

    bool armPresent;

    struct{
        bool active;
        int port;
    }sockTCP;

    struct{
        int safe_curr_limit;
        int peak_curr;
        int cont_curr;
        int max_speed;
    }homeConfig;

    struct{
        int safe_curr_limit;
        int peak_curr;
        int cont_curr;
        int max_speed;
        int max_acc;
        int max_deacc;
    }operConfig;

    int finger_conf_num;
    double finger_confs[MAX_CONF][NUM_MOT];

    bool TcpActive;

    unsigned int DOF_;
    unsigned int nfingers;
    bool isHomeDone;

    bool doHome;
    int auxInternalCall;
    bool fault;
    bool tcpDoHome;
    bool tcpIsHomeDone;
    bool setTactOffset;

    /********   RosInterface Stuffs  ********/
#ifdef ROS_IF

#endif
    // --- input variables from the RosInterface  --- //
    int srv_preshape;
    double srv_radius;
    double	srv_radius_palm;
    float action_force;
    int action_goal;
    int srv_mode;
    bool emerg_stop;

    // --- Output variables to the RosInterface  --- //
    double action_perc_complete;
    bool action_done;
    bool action_free;

    int fingerOffset;

    /*****************************************/
};

#endif // CONTROLLERDATA_H
