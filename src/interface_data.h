/*
 * interface_data.h
 *
 *  Created on: Aug 29, 2015
 *      Author: Gianluca Palli
 */

#ifndef INTERFACE_DATA_H_
#define INTERFACE_DATA_H_

#include "controllerdata.h"
#include "can_define.h"

#define RAD_TO_GRAD			(180/M_PI)
#define GRAD_TO_RAD			(M_PI/180)

#define BIT16_	1U<<15	//0x8000
#define BIT32_	1U<<31	//0x80000000
#define ERR_VAL BIT32_

//#define HOME_MAXPEAK				3000
//#define HOME_MAXCONT				2300
//#define MAX_GRASPCURR				10000	//maximo assorbimento totale
//#define NUM_MOT	3
//#define STDMAXVEL					3000

#pragma pack(8)

/* struct aggiormanento interfaccia */

typedef struct
{
    int JointID[NUM_MOT];

    int State[NUM_MOT];

    double Velocity[NUM_MOT];	//velocita attuale
    long long Current[NUM_MOT];	//corrente attuale
    long long Control[NUM_MOT];		//velocita impostata
    double PositionGrad[NUM_MOT];	//posizione in gradi
    double MaxPosGrad[NUM_MOT];	//massima apertura

    bool Operational[NUM_MOT];
    bool MotorFault[NUM_MOT];
    bool isHomeDone;
    bool SystemFault;
    //bool isInitialized;

    //std::string strStatus;

    /* ***** strees test stuff ** */

    long long actCycle;

    /*******************************/

    int homePhase;
    int initPhase;
    int srv_mode;
    int srv_preshape;

    bool isInitialized_;
    bool emerg_stop;
    bool resetSensors;

    bool fake_align[12];	//align with 64 bit grafic interface
} SystemStatus;

/* ****************************** */
#define DO_NOTHING      0
#define GO_POSITION     1
#define GO_VELOCITY     2
#define RECOVER         3
#define SET_INIT_POS    4
#define SET_FINAL_POS   5
#define EMERGENCY       6
#define GO_FINAL_POS    7
#define PRESHAPE        8
#define STOP            9
#define RUN             10
#define SET_HOME        11

/* struct setting (user -> control) */
typedef struct
{
    uint64_t command;
    uint64_t preshape;
    union
    {
        int64_t req_pos[NUM_MOT];
        int64_t req_vel[NUM_MOT];
        bool motor_selection[NUM_MOT];
    };

#ifdef ARCH_32BIT
    //int fake_align[2];	//align with 64 bit grafic interface
#endif
} SystemRequest;

#endif /* INTERFACE_DATA_H_ */

