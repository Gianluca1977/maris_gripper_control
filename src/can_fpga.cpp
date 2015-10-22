#include "can_fpga.h"
#include "can_driver.h"

std::queue<TPCANMsg> CanFpga::msg_inqueue;

WF::Task* CanFpga::writing_task;
WF::Task* CanFpga::reading_task;

WF::Thread CanFpga::writing_thr;
WF::Thread CanFpga::reading_thr;
void* CanFpga::returnValue;

CanFpga::CanFpga()
{

    char *ptr;
    int i;
    int nType = HW_PCI;
    __u32 dwPort = 0;
    __u16 wIrq = 0;
    __u16 wBTR0BTR1 = 0;
    char *filename = NULL;
    const char *szDevNode = DEFAULT_NODE;
    bool bDevNodeGiven = false;
    bool bTypeGiven = false;

    char txt[VERSIONSTRING_LEN];

    nExtended = CAN_INIT_TYPE_ST;

    wBTR0BTR1 = 0x0014;

    if (wBTR0BTR1)
        printf(", init with BTR0BTR1=0x%04x\n", wBTR0BTR1);
    else
        printf(", init with 500 kbit/sec.\n");
    printf("             Data will be read from \"%s\".\n", filename);

    /* open CAN port */
    if ((bDevNodeGiven) || (!bDevNodeGiven && !bTypeGiven)) {
        h = LINUX_CAN_Open(szDevNode, O_RDWR);
        if (h)
            SET_INIT_STATE(STATE_FILE_OPENED);
        else {
            printf("can_interface_rt: can't open %s\n", szDevNode);
            goto error;
        }
    }
    else {
        // please use what is appropriate
        // HW_DONGLE_SJA
        // HW_DONGLE_SJA_EPP
        // HW_ISA_SJA
        // HW_PCI
        h = CAN_Open(nType, dwPort, wIrq);
        if (h)
            SET_INIT_STATE(STATE_FILE_OPENED);
        else {
            printf("can_interface_rt: can't open %s device.\n", getNameOfInterface(nType));
            goto error;
        }
    }

    /* clear status */
    CAN_Status(h);

    // get version info
    errno = CAN_VersionInfo(h, txt);
    if (!errno)
        printf("can_interface_rt: driver version = %s\n", txt);
    else {
        perror("can_interface_rt: CAN_VersionInfo()");
        goto error;
    }

    // init to a user defined bit rate
    if (wBTR0BTR1) {
        errno = CAN_Init(h, wBTR0BTR1, nExtended);
        if (errno) {
            perror("can_interface_rt: CAN_Init()");
            goto error;
        }
    }

    reading_thr.Create(reading_task_proc, this);
    writing_thr.Create(writing_task_proc, this);

    SET_INIT_STATE(STATE_TASK_CREATED);
}

CanFpga::~CanFpga()
{

    if (current_state & STATE_FILE_OPENED) {
        printf("can_interface_rt");
        CAN_Close(h);
        RESET_INIT_STATE(STATE_FILE_OPENED);
    }

    writing_thr.Join();
    reading_thr.Join();

    if (current_state & STATE_TASK_CREATED) {
        RESET_INIT_STATE(STATE_TASK_CREATED);
    }

}

//----------------------------------------------------------------------------
// real time task
void* CanFpga::reading_task_proc(void *arg)
{
    TPCANMsg msg;

    CanFpga *current_if = (CanFpga*) arg;

    __u32 status;

    int ret = 0;
    //action_done = false;

    /* task initialization */
    reading_task = WF::Task::GetInstance();
    if ((ret=reading_task->CreateAsync(READING_FPGATASK_NAME, WF_TASK_TYPE_USER, READING_FPGATASK_PRIORITY)) != WF_RV_OK)
    {
        KAL::DebugConsole::Write(LOG_LEVEL_ERROR, READING_FPGATASK_NAME, "CreateSync fallita. (valore ritorno %d)", ret);
        WF::Task::Exit();
    }

    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, READING_FPGATASK_NAME, "CanFpga::reading_task_proc Created");// %s %p", ST_name, this);

    reading_task->SetReadyUntilPostInit();
    reading_task->WaitRunning();

    // Enter hard realtime
    reading_task->GotoHard();

    while (reading_task->Continue() && !GLOBALSTOP) {
        //printf("in while\n");

        if ((errno = CAN_Read(h, &msg))){
            shutdownnow = 1;
        }
        else {
            // check if a CAN status is pending
            if (msg.MSGTYPE & MSGTYPE_STATUS) {
                status = CAN_Status(h);
                if ((int)status < 0){
                    shutdownnow = 1;
                }
                //  else print_can_msg(&m);
            }
        }

        print_can_msg(&msg);
        ret = rt_mbx_send(out_mbx,&msg,sizeof(TPCANMsg));
        if(ret < 0){
            printf("reading_task_proc: rt_mbx_send error\n");
            shutdownnow = 1;
        }

        if (shutdownnow == 1) break;
    }

    reading_task->GotoSoft();
    reading_task->Release();
    WF::Task::Exit();

    returnValue = (void *) WF_RV_OK;

}

void* CanFpga::writing_task_proc(void *arg)
{
    TPCANMsg msg;

    CanFpga *current_if = (CanFpga*) arg;

    int ret = 0;
    //action_done = false;

    /* task initialization */
    writing_task = WF::Task::GetInstance();
    if ((ret=writing_task->CreateSync(WRITING_FPGATASK_NAME, WRITING_FPGATASK_SAMPLETIME, WF_TASK_TYPE_USER)) != WF_RV_OK)
    {
        KAL::DebugConsole::Write(LOG_LEVEL_ERROR, WRITING_FPGATASK_NAME, "CreateSync fallita. (valore ritorno %d)", ret);
        WF::Task::Exit();
    }

    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, WRITING_FPGATASK_NAME, "CanFpga::writing_task_proc Created");// %s %p", ST_name, this);

    writing_task->SetReadyUntilPostInit();
    writing_task->WaitRunning();

    // Enter hard realtime
    writing_task->GotoHard();


    //---------------
    while (writing_task->Continue() && !GLOBALSTOP) {

        ret = rt_mbx_receive(in_mbx,&msg,sizeof(TPCANMsg));
        if(ret < 0){
            printf("writing_task_proc: rt_mbx_receive error\n");
            shutdownnow = 1;
            break;
        }

        print_can_msg(msg);

        // test for standard frames only
        if ((nExtended == CAN_INIT_TYPE_EX) || !(iter->MSGTYPE & MSGTYPE_EXTENDED)) {
            // send the message
            if ((errno = CAN_Write(h, &(*iter))))
                shutdownnow = 1;
        }
        if (shutdownnow == 1) break;

    }

    writing_task->GotoSoft();
    writing_task->Release();
    WF::Task::Exit();

    returnValue = (void *) WF_RV_OK;

}


