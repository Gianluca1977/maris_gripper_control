//============================================================================
// Name        : main.cpp
// Author      : Gianluca Palli
// Version     :
// Copyright   :
// Description :
//============================================================================

//#define ROS_IF

#ifdef ROS_IF
#include "ros_interface.h"
#endif

#include "main.h"

#include "can_driver.h"
#include "interface_data.h"
#include "tcp_interface.h"
#include "controller.h"

#include "wf.h"

static WF::Task *maintask;

int initPhase = INIT_BOOTUP_DEV;

void main_shutdown(Controller* gripper){
    maintask->GotoSoft();
    maintask->Release();

#ifdef ROS_IF
    //delete gripper->rosInter;
#endif

    delete gripper;
}

void main_shutdown()
{
    maintask->GotoSoft();
    maintask->Release();

#ifdef ROS_IF
    //delete gripper->rosInter;
#endif
}

int main(int argc, char** argv)
{
    TPCANMsg msg;

    int ret;   

    //si ottiene il singleton per la gestione del task
    maintask = WF::Task::GetInstance();

    if (maintask->CreateAsync(MAINTASK_NAME, WF_TASK_TYPE_USER, MAINTASK_PRIORITY) != WF_RV_OK){
        std::cerr << "Could not initialize " << MAINTASK_NAME << std::endl;        
        exit(-1);
    }

    Controller gripper_control;

    maintask->GotoHard();

    ret = maintask->WaitRunning();
    if (ret != WF_RV_OK){
        KAL::DebugConsole::Write(LOG_LEVEL_ERROR, MAINTASK_NAME, "WaitRunning fallita. (valore ritorno %d)", ret);
        main_shutdown();
        return WF_RV_FAIL;
    }

    int canRet;

    // Main loop
    while (maintask->Continue() && !gripper_control.GLOBALSTOP){
        //KAL::DebugConsole::Write(LOG_LEVEL_INFO, MAINTASK_NAME, "Waiting for CAN packets...");
        canRet = gripper_control.receive_can_msg(msg);
#ifdef VM_TEST
        maintask->Sleep(WF_TIME_ONE_S);
#endif

        //KAL::DebugConsole::Write(LOG_LEVEL_INFO, MAINTASK_NAME, "Exit from  schedHelper_.Receive.");

        if (canRet == WF_RV_FAIL) {
            KAL::DebugConsole::Write(LOG_LEVEL_WARNING, MAINTASK_NAME,"Inconsistent CAN msg lenght");
            main_shutdown();
            return WF_RV_FAIL;
        } else {
            //KAL::DebugConsole::Write(LOG_LEVEL_INFO, MAINTASK_NAME, "Packet Received.");
            if (canRet == WF_RV_SHUTDOWN) {
                if (maintask->IsKilled() == true) {
                    KAL::DebugConsole::Write(LOG_LEVEL_WARNING, MAINTASK_NAME, "System is killing down, forcibly closing.");
                    main_shutdown();
                    return WF_RV_FAIL;
                }
                if (maintask->ShuttingDown() == true) {
                    KAL::DebugConsole::Write(LOG_LEVEL_WARNING, MAINTASK_NAME, "System is shutting down, gracefully closing.");
                    main_shutdown();
                    return WF_RV_OK;
                }
            } else {

                //KAL::DebugConsole::Write(LOG_LEVEL_INFO, MAINTASK_NAME, "CAN msg Received ID = %03X TYPE = %02X LEN = %d DATA = %02X %02X %02X %02X %02X %02X %02X %02X", tmpCANMsg.msg.ID, tmpCANMsg.msg.MSGTYPE, tmpCANMsg.msg.LEN, tmpCANMsg.msg.DATA[0], tmpCANMsg.msg.DATA[1], tmpCANMsg.msg.DATA[2], tmpCANMsg.msg.DATA[3], tmpCANMsg.msg.DATA[4], tmpCANMsg.msg.DATA[5], tmpCANMsg.msg.DATA[6], tmpCANMsg.msg.DATA[7]);

                gripper_control.process_message(msg);

                //} else KAL::DebugConsole::Write(LOG_LEVEL_WARNING,MAINTASK_NAME,"Inconsistent CAN msg lenght");
            }
        }
    }//while (maintask->Continue() && !GLOBALSTOP)

    main_shutdown();

    maintask->Exit();

    std::cout << "Gripper App finished" << std::endl;

    return 0;
}
