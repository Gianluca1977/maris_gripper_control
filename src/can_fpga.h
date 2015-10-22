#ifndef CAN_FPGA_H
#define CAN_FPGA_H

//----------------------------------------------------------------------------
// set here current release for this program
#define CURRENT_RELEASE "0.1"

//****************************************************************************
// INCLUDES

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>   // exit
#include <signal.h>
#include <string.h>
#include <stdlib.h>   // strtoul
#include <fcntl.h>    // O_RDWR

#include <libpcan.h>
#include <src/common.h>
#include <ctype.h>
#include <src/parser.h>

#include "can_driver.h"
#include "supervisor.h"

//****************************************************************************
// DEFINES

#define DEFAULT_NODE "/dev/pcan1"
#define MAX_MSG 20

#define WRITING_FPGATASK_NAME    WRTSK
#define READING_FPGATASK_NAME    RDTSK

#define WRITING_FPGATASK_SAMPLETIME (10 * WF_TIME_ONE_MS)
#define READING_FPGATASK_PRIORITY   (10)

//****************************************************************************
// INCLUDES

#include <sys/mman.h>


#include <sys/poll.h>

//****************************************************************************
// DEFINES

#define STATE_FILE_OPENED         1
#define STATE_TASK_CREATED        2
#define SET_INIT_STATE(new_state) current_state |= new_state
#define RESET_INIT_STATE(new_state) current_state &= ~new_state

class CanFpga : private CanInterface, virtual protected Supervisor
{

    HANDLE h;
    const char *current_release;
    int nExtended;


    static std::queue<TPCANMsg> msg_inqueue; // in msg queue (from the devices)

    //****************************************************************************
    // GLOBALS
    unsigned int current_state = 0;
    int shutdownnow = 0;

    static WF::Task *writing_task;
    static WF::Task *reading_task;

    static WF::Thread writing_thr;
    static WF::Thread reading_thr;
    static void* returnValue;

public:
    CanFpga();
    ~CanFpga();

    void *reading_task_proc(void *arg);
    void *writing_task_proc(void *arg);
};

#endif // CAN_FPGA_H
