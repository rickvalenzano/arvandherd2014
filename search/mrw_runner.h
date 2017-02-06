#ifndef MRW_RUNNER_H
#define MRW_RUNNER_H

#include <pthread.h>
#include "globals.h"
#include "string.h"

struct mrw_thread_data{
   int  seed;
   string name;
};

void run_mrw_search(bool other_threads);
void *run_mrw_thread(void *data);

#endif
