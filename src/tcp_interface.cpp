#include "tcp_interface.h"
#include "interface_data.h"

extern bool GLOBALSTOP;

int TcpData::sockfd;
int TcpData::newsockfd;

int TcpData::port;

socklen_t TcpData::clilen;
struct sockaddr_in TcpData::serv_addr, TcpData::cli_addr;
struct hostent* TcpData::server;

SystemStatus TcpData::Status;
SystemRequest TcpData::Request;

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

TcpInterface::TcpInterface() //: TcpData(portno)
{
}

TcpInterface::~TcpInterface()
{
}

TcpReceive::TcpReceive()
{
    //init the Request struct with non valid values
    Request.command = DO_NOTHING;

    for(unsigned int i=0;i<NUM_MOT;i++)
    {
        Request.req_pos[i] = ERR_VAL;
        Request.req_vel[i] = 0;
    }

    //KAL::DebugConsole::Write(LOG_LEVEL_INFO, TCPRECVTASK_NAME, "Before creating TcpReceive");
    if_thread.Create(thread_func, this);
}

TcpSend::TcpSend()
{
    for(unsigned int f = 0; f < NUM_MOT; f++)
    {
        Status.JointID[f] = nodeIds[f];	//riempio il vettore di ID della interfaccia
    }

    Status.actCycle = 0;

    //KAL::DebugConsole::Write(LOG_LEVEL_INFO, TCPSENDTASK_NAME, "Before creating TcpSend");
    if_thread.Create(thread_func, this);
}

void TcpReceive::rt_thread_handler()
{
    /*
    *  aggiornamento interfaccia grafica (grafic -> control)
    */
    int ret;

    /* task initialization */
    if_task = WF::Task::GetInstance();

    if((ret=if_task->CreateAsync(TCPRECVTASK_NAME, WF_TASK_TYPE_USER, TCPRECVTASK_PRIORITY)) != WF_RV_OK)  {
        KAL::DebugConsole::Write(LOG_LEVEL_ERROR, TCPRECVTASK_NAME, "CreateSync fallita. (valore ritorno %d)", ret);
        WF::Task::Exit();
    }

    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, TCPRECVTASK_NAME, "TcpReceive Created on port %d", port);

    if_task->SetReadyUntilPostInit();

    /* ************************* */

    if_task->WaitRunning();

    TcpActive = false;

    while (if_task->Continue() && !GLOBALSTOP)
    {
        // ricevere commandi dalla interfaccia

        if(!TcpActive)
        {
            KAL::DebugConsole::Write(LOG_LEVEL_WARNING, TCPRECVTASK_NAME, "Waiting for incoming TCP connections");

            /* SETTING UP TCP SOCKET */
            sockfd = socket(AF_INET, SOCK_STREAM, 0);
            if (sockfd < 0){
                error("ERROR OPENING SOCKET");
            }
            bzero((char *) &serv_addr, sizeof(serv_addr));
            //    portno = port;

            serv_addr.sin_family = AF_INET;
            serv_addr.sin_addr.s_addr = INADDR_ANY;
            serv_addr.sin_port = htons(port);

            clilen = sizeof(cli_addr);

            // set SO_REUSEADDR on a socket to true (1):
            int optval = 1;
            setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);

            if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0){
                error("ERROR ON BINDING");
            }

            ret = listen(sockfd,5);

            KAL::DebugConsole::Write(LOG_LEVEL_WARNING, TCPRECVTASK_NAME, "listen returned %d", ret);

            newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);

            KAL::DebugConsole::Write(LOG_LEVEL_WARNING, TCPRECVTASK_NAME, "after accept");
            if (newsockfd < 0){
                //error("ERROR ON ACCEPT");
            } else {
                std::cout << "CONNECTION ACCEPT" << std::endl;
                TcpActive = true;
            }
        }

        if(TcpActive){
            KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, TCPRECVTASK_NAME, "Waiting for incoming TCP data");

            ret = MsgSem.Wait();
            if(ret != WF_RV_OK) KAL::DebugConsole::Write(LOG_LEVEL_ERROR, TCPRECVTASK_NAME, "Error in MsgSem.Signal()");

            ret = read(newsockfd,&Request,sizeof(SystemRequest));

            if(ret == sizeof(SystemRequest))
            {
                //#ifdef _DEBUG_
                KAL::DebugConsole::Write(LOG_LEVEL_INFO, TCPRECVTASK_NAME, "Command %d",Request.command);
                KAL::DebugConsole::Write(LOG_LEVEL_INFO, TCPRECVTASK_NAME, "preshape %d",Request.preshape);
                KAL::DebugConsole::Write(LOG_LEVEL_INFO, TCPRECVTASK_NAME, "req_vel = %lld %lld %lld",Request.req_vel[0],Request.req_vel[1],Request.req_vel[2]);
                KAL::DebugConsole::Write(LOG_LEVEL_INFO, TCPRECVTASK_NAME, "req_pos = %lld %lld %lld",Request.req_pos[0],Request.req_pos[1],Request.req_pos[2]);
//                KAL::DebugConsole::Write(LOG_LEVEL_INFO, TCPRECVTASK_NAME, "repeat = %lld, butNum = %lld",Request.repeat,Request.butNum);
//                KAL::DebugConsole::Write(LOG_LEVEL_INFO, TCPRECVTASK_NAME, "doHome = %d, parking = %d",Request.doHome,Request.parking);
//                KAL::DebugConsole::Write(LOG_LEVEL_INFO, TCPRECVTASK_NAME, "emerg_stop = %d",Request.emerg_stop);
//                KAL::DebugConsole::Write(LOG_LEVEL_INFO, TCPRECVTASK_NAME, "manualHomeDone = %d",Request.manualHomeDone);
//                KAL::DebugConsole::Write(LOG_LEVEL_INFO, TCPRECVTASK_NAME, "tcpActive = %d",Request.tcpActive);
                //#endif                
            }
            else
            {
                Request.command = DO_NOTHING;

                KAL::DebugConsole::Write(LOG_LEVEL_ERROR, TCPRECVTASK_NAME, "Dimensione NON Corretta %d",ret);
                TcpActive = false;
                if(ret == 0)
                {
                    KAL::DebugConsole::Write(LOG_LEVEL_INFO, TCPRECVTASK_NAME, "EOF reached");
                }
                else if(ret<0)
                {
                    KAL::DebugConsole::Write(LOG_LEVEL_ERROR, TCPRECVTASK_NAME, "read generated an error %d", ret);
                    perror("TcpReceive::rt_thread_handler");                    
                }
                shutdown(newsockfd,2);
                shutdown(sockfd,2);
                close(newsockfd);
                close(sockfd);
            }

            ret = MsgSem.Signal();
            if(ret != WF_RV_OK) KAL::DebugConsole::Write(LOG_LEVEL_ERROR, TCPRECVTASK_NAME, "Error in MsgSem.Signal()");

            ret = if_task->Sleep(100 * WF_TIME_ONE_MS);
            if(ret != WF_RV_OK) KAL::DebugConsole::Write(LOG_LEVEL_ERROR, TCPRECVTASK_NAME, "Error in if_task->Sleep()");

//            else
//            {
//                KAL::DebugConsole::Write(LOG_LEVEL_ERROR, TCPRECVTASK_NAME, "Unable to lock MSGSEM");
//            }
        }
    }
    // Enable this if your task was Hard Real Time
    //delete Request;
    shutdown(newsockfd,2);
    shutdown(sockfd,2);
    close(newsockfd);
    close(sockfd);
    if_task->Release();
    WF::Task::Exit();

    returnValue = (void *) WF_RV_OK;
}

void TcpSend::rt_thread_handler()
{
    /*
    * aggiornamento interfaccia grafica (control -> grafic)
    */
    int ret;
    int n;

    /* task initialization */
    if_task = WF::Task::GetInstance();
    if ((ret=if_task->CreateSync(TCPSENDTASK_NAME, TCPSEND_SAMPLETIME, WF_TASK_TYPE_USER)) != WF_RV_OK){
        KAL::DebugConsole::Write(LOG_LEVEL_ERROR, TCPSENDTASK_NAME, "CreateSync fallita. (valore ritorno %d)", ret);
        WF::Task::Exit();
    }

    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, TCPSENDTASK_NAME, "TcpSend Created %d", sizeof(SystemStatus));

    if_task->SetReadyUntilPostInit();
    if_task->WaitRunning();

    while (if_task->Continue() && !GLOBALSTOP){
        //inviare dati alla maixbox grafica
        if(TcpActive) n = write(newsockfd,&Status,sizeof(SystemStatus));
        if_task->WaitPeriod();

    }
    // Enable this if your task was Hard Real Time
    // delete Status;
    if_task->Release();
    WF::Task::Exit();

    returnValue = (void *) WF_RV_OK;
}

