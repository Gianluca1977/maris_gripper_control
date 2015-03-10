/*
 * tcpServer.h
 *
 *  Created on: Sep 13, 2012
 *      Author: lar
 */

#ifndef TCPSERVER_H_
#define TCPSERVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#include "interface_data.h"
#include "wf.h"
#include "can_driver.h"
#include "supervisor.h"
#include "rt_thread.h"
#include "controllerdata.h"

#define TCPRECVTASK_NAME    ("TCP_TASK_RECV")
#define TCPSENDTASK_NAME    ("TCP_TASK_SEND")
//#define TCPSEND_SAMPLETIME  (50 * WF_TIME_ONE_MS)
#define TCPSEND_SAMPLETIME  (100 * WF_TIME_ONE_MS)
#define TCPRECVTASK_PRIORITY	(10)

class TcpData : virtual protected ControllerData
{
public:    
    static SystemRequest Request;
    static SystemStatus Status;

    //TcpData(int portno){port = portno;};
    TcpData(){port = sockTCP.port;}

protected:

    static int port;

    static int sockfd, newsockfd;
    static socklen_t clilen;
    static struct sockaddr_in serv_addr, cli_addr;
    static struct hostent *server;
};

class TcpSend : virtual protected TcpData, virtual protected Supervisor, private rt_thread
{
public:
    TcpSend();

private:
    void rt_thread_handler(void);
};

class TcpReceive : virtual protected TcpData, virtual protected Supervisor, private rt_thread
{
public:
    TcpReceive();

private:
    void rt_thread_handler(void);
};

class TcpInterface : public TcpSend, public TcpReceive
{
public:
    TcpInterface();
    ~TcpInterface();
};



#endif /* TCPSERVER_H_ */
