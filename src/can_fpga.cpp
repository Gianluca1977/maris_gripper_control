#include "can_fpga.h"
#include "can_driver.h"

std::queue<TPCANMsg> CanFpga::msg_inqueue;

//WF::CAN CanFpga::mycan;

WF::Task* CanFpga::writing_task;
WF::Task* CanFpga::reading_task;

WF::Thread CanFpga::writing_thr;
WF::Thread CanFpga::reading_thr;
void* CanFpga::returnValue;

CanFpga::CanFpga()
{

    char *ptr;
    int i;
    int fd;
    int nType = HW_PCI;
    __u32 dwPort = 0;
    __u16 wIrq = 0;
    __u16 wBTR0BTR1 = 0;
    char *filename = NULL;
    const char *szDevNode = DEFAULT_NODE;
    bool bDevNodeGiven = true;
    bool bTypeGiven = false;

    char txt[VERSIONSTRING_LEN];

    current_state = 0;

    nExtended = CAN_INIT_TYPE_ST;

    wBTR0BTR1 = 0x0014;

    if (wBTR0BTR1)
        printf(", init with BTR0BTR1=0x%04x\n", wBTR0BTR1);
    else
        printf(", init with 500 kbit/sec.\n");
    printf("             Data will be read from \"%s\".\n", filename);

    //fd = mycan.Open("/dev/pcan1", O_RDWR, wBTR0BTR1);
    //printf("can_interface_rt: mycan.Open returns %d\n", ret);

    /* open CAN port */
    if ((bDevNodeGiven) || (!bDevNodeGiven && !bTypeGiven)) {
        h = LINUX_CAN_Open(szDevNode, O_RDWR);
        if (h)
            SET_INIT_STATE(STATE_FILE_OPENED);
        else {
            printf("can_interface_rt: can't open %s\n", szDevNode);
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
    }

    // init to a user defined bit rate
    if (wBTR0BTR1) {
        errno = CAN_Init(h, wBTR0BTR1, nExtended);
        if (errno) {
            perror("can_interface_rt: CAN_Init()");
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
    //reading_task->GotoHard();

    while (reading_task->Continue() && !GLOBALSTOP) {
        //printf("in while\n");

        if ((errno = CAN_Read(current_if->h, &msg))){
            KAL::DebugConsole::Write(LOG_LEVEL_ERROR, "FPGA_reading_task", "CAN_Read return %d",errno);
        }
        else {
            // check if a CAN status is pending
            if (msg.MSGTYPE & MSGTYPE_STATUS) {
                status = CAN_Status(current_if->h);
                if ((int)status < 0){
                    KAL::DebugConsole::Write(LOG_LEVEL_ERROR, "FPGA_reading_task", "reading_task_proc: status is negative");
                }
                //else print_can_msg(&m);
            }
        }

        KAL::DebugConsole::Write(LOG_LEVEL_INFO, "FPGA_reading_task", "CAN msg Received ID = %03X TYPE = %02X LEN = %d DATA = %02X %02X %02X %02X %02X %02X %02X %02X", msg.ID, msg.MSGTYPE, msg.LEN, msg.DATA[0], msg.DATA[1], msg.DATA[2], msg.DATA[3], msg.DATA[4], msg.DATA[5], msg.DATA[6], msg.DATA[7]);

        msg_inqueue.push(msg);

    }

    reading_task->GotoSoft();
    reading_task->Release();
    WF::Task::Exit();

    returnValue = (void *) WF_RV_OK;

}

void* CanFpga::writing_task_proc(void *arg)
{
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
    //writing_task->GotoHard();

    //---------------
    while (writing_task->Continue() && !GLOBALSTOP) {

        while(!msg_outqueue.empty()){
            TPCANMsg* CANMsg = &(msg_outqueue.front());
            KAL::DebugConsole::Write(LOG_LEVEL_INFO, "FPGA_writing_task", "Send CAN msg ID = %03X TYPE = %02X LEN = %d DATA = %02X %02X %02X %02X %02X %02X %02X %02X", CANMsg->ID, CANMsg->MSGTYPE, CANMsg->LEN, CANMsg->DATA[0], CANMsg->DATA[1], CANMsg->DATA[2], CANMsg->DATA[3], CANMsg->DATA[4], CANMsg->DATA[5], CANMsg->DATA[6], CANMsg->DATA[7]);

#ifndef VM_TEST
            if ((errno = CAN_Write(current_if->h, CANMsg)))
                KAL::DebugConsole::Write(LOG_LEVEL_ERROR, "FPGA_writing_task", "CAN_Write return %d",errno);

            rt_sleep(WF_TIME_ONE_MS);
#endif

            msg_outqueue.pop();
        }


    }

    writing_task->GotoSoft();
    writing_task->Release();
    WF::Task::Exit();

    returnValue = (void *) WF_RV_OK;

}

// lookup for HW_... constant out of device type string
int CanFpga::getTypeOfInterface(char *szTypeName)
{
    int nType = 0;

    if (!strcmp(szTypeName, "pci"))
        nType = HW_PCI;
    else
    {
        if (!strcmp(szTypeName, "isa"))
            nType = HW_ISA_SJA;
        else
        {
            if (!strcmp(szTypeName, "sp"))
                nType = HW_DONGLE_SJA;
            else
            {
                if (!strcmp(szTypeName, "epp"))
                    nType = HW_DONGLE_SJA_EPP;
                else
                {
                    if (!strcmp(szTypeName, "usb"))
                        nType = HW_USB;
                    else
                    {
                        if (!strcmp(szTypeName, "usbpro"))
                            nType = HW_USB_PRO;
                        else
                        {
                            if (!strcmp(szTypeName, "pccard"))
                                nType = HW_PCCARD;
                        }
                    }
                }
            }
        }
    }

    return nType;
}

// the opposite: lookup for device string out of HW_.. constant
char *CanFpga::getNameOfInterface(int nType)
{
  switch (nType)
  {
    case HW_PCI:            return "pci";
    case HW_ISA_SJA:        return "isa";
    case HW_DONGLE_SJA:     return "sp";
    case HW_DONGLE_SJA_EPP: return "epp";
    case HW_USB:            return "usb";
    case HW_USB_PRO:        return "usbpro";
    case HW_PCCARD:         return "pccard";

    default:                return "unknown";
  }
}
