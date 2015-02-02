#include "rt_thread.h"

/* real time thread wrapper */
void *rt_thread::thread_func(void *p){

  rt_thread *tmp_thread = (rt_thread *)p;

  //DEBUG("thread_func invoked by %s\n",tmp_thread->get_label());

  tmp_thread->rt_thread_handler();

  //DEBUG("%s thread_func terminated\n",tmp_thread->get_label());

  return tmp_thread->returnValue;
}

/***************************************************************************

real time thread destructor

 ***************************************************************************/

rt_thread::~rt_thread(void){

  if_thread.Join();
  //DEBUG("%s thread joined\n",get_label());
}
