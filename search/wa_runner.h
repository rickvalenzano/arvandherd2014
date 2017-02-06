#ifndef WA_RUNNER_H
#define WA_RUNNER_H

#include "globals.h"
#include "wa_star_params.h"

// Takes in the wa_star_params to run and whether we should run a mrw instance
// after the WA* run is done
void run_wa_star(WA_Star_Parameters *wa_star_params, bool run_mrw_after);

#endif

