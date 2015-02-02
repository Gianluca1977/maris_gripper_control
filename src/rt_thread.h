#ifndef RT_THREAD_H
#define RT_THREAD_H

#include "wf.h"
#include "supervisor.h"

#define MAX_NAMES_SIZE	32

class rt_thread : virtual private Supervisor
{    
    char if_label[MAX_NAMES_SIZE];

protected:
    WF::Thread if_thread;
    WF::Task *if_task;
    void* returnValue;

public:
    rt_thread(){//if_thread.Create(thread_func, this);
                returnValue = NULL;}
    virtual ~rt_thread(void);
    virtual void rt_thread_handler(void)=0;
    static void *thread_func(void* p);
    //void set_label(char *str){sprintf(if_label,str);}
    //char* get_label(void){return if_label;}
};

#endif // RT_THREAD_H
