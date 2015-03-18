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

    /**
         * @brief Create thread which will execute function
         * @param function function to execute
         * @param arg argument to pass to function
         * @return WF_RV_FAIL error on thread creation or already created thread
         * @return WF_RV_OK success
         */
    int rt_thread_create()
    {
        return if_thread.Create(thread_func, this);
    }

    /**
     * @brief wait for termination of the thread
     * @return WF_RV_FAIL error on thread join
     * @return WF_RV_OK success
     */
    int rt_thread_join()
    {
        return if_thread.Join();
    }

    virtual ~rt_thread(void);
    virtual void rt_thread_handler(void)=0;
    static void *thread_func(void* p);
    //void set_label(char *str){sprintf(if_label,str);}
    //char* get_label(void){return if_label;}
};

#endif // RT_THREAD_H
