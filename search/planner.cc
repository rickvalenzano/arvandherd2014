/*********************************************************************
 * Author: Malte Helmert (helmert@informatik.uni-freiburg.de)
 * (C) Copyright 2003-2004 Malte Helmert
 * Modified by: Silvia Richter (silvia.richter@nicta.com.au),
 *              Matthias Westphal (westpham@informatik.uni-freiburg.de)             
 * (C) Copyright 2008 NICTA and Matthias Westphal
 * This file is part of LAMA.
 *
 * LAMA is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 3
 * of the license, or (at your option) any later version.
 *
 * LAMA is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 *********************************************************************/

#include "globals.h"
#include "operator.h"
#include "string.h"
#include "wa_runner.h"
#include "mrw_runner.h"

#include "command_line_parsing.h"

#include <cassert>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <sys/times.h>
#include <climits>

using namespace std;

bool are_generating_landmarks(WA_Star_Parameters *wa_star_params);

int main(int argc, const char **argv) {
    
    bool poly_time_method = false; 
    bool reasonable_orders = true; // still don't know what his does

    WA_Star_Parameters *wa_star_params = NULL; // WA* params

    // Processes command line
	if (!process_command_line(argc, argv, wa_star_params)) {
	    cerr << "use -help (and only -help) for information" << endl;
        exit(1);
    }

	// Set mrw shared, but not mrw, so add default mrw
	if(g_params_list.size() == 0 && g_mrw_shared != NULL){
		cout << "Using single default version of MRW" << endl << endl;
		MRW_Parameters* default_mrw_params = new MRW_Parameters();
		g_params_list.push_back(default_mrw_params);
	} else if(g_params_list.size() > 0 && g_mrw_shared == NULL) {
		cout << "Using default version of shared MRW" << endl << endl;
		g_mrw_shared = new Shared_MRW_Parameters;
	} else if(wa_star_params == NULL) { // Run MRW as default if no stdin values
		cout << "Using single default version of MRW" << endl << endl;
		MRW_Parameters* default_mrw_params = new MRW_Parameters();
		g_params_list.push_back(default_mrw_params);
		cout << "Using default version of shared MRW" << endl << endl;
		g_mrw_shared = new Shared_MRW_Parameters;
	}

    // ******* Print out configuration information *******
    output_shared_param_values();
    cout << endl;
    
    // prints out restart info if doing mrw run
    if(!g_params_list.empty()) {
    	cout << "MRW Parameters" << endl;
    	g_mrw_shared->print_values();
    	cout << endl ;
    }
    // prints out mrw configurations information
    for(int i = 0; i < g_params_list.size(); i++) {
        cout << "\tMRW Param " << i << ":" << endl;
        g_params_list[i]->print_values();
        cout << endl;
    }

    if(wa_star_params != NULL) {
        cout << "WA* Params" << endl;
        wa_star_params->print_values();
        cout << endl << endl;
    }

    // what does this do?
	cin >> poly_time_method;
	if (poly_time_method) {
		cout << "Poly-time method not implemented in this branch." << endl;
		cout << "Starting normal solver." << endl;
	}

	// Read input and generate landmarks
	bool generate_landmarks = are_generating_landmarks(wa_star_params);
	g_lgraph = NULL;
	Timer landmark_timer;
	read_everything(cin, generate_landmarks, reasonable_orders);
    landmark_timer.stop();
	if(g_lgraph != NULL) 
		cout << "Landmarks generation time: " << landmark_timer << endl;

	/* Still need to do something with this
	if(g_init_trajectory_file != NULL &&
	    !parse_trajectory_file(g_init_trajectory_file)) {
	    cerr << "Trajectory File Reading failed" << endl;
	    exit(1);
	}

	// TODO Test more
	State s(*g_initial_state);
	for(int i = 0; i < g_init_trajectory.size(); i++) {

	    State next_state(s, *g_init_trajectory[i]);
        s = next_state;
	}

	g_initial_state = &s;
	*/

	bool finish_mrw = (wa_star_params == NULL ||
			(g_mrw_shared != NULL && g_mrw_shared->mrw_time_limit > 0));

	// performs monte carlo random walks
	if(!g_params_list.empty())
        run_mrw_search(finish_mrw);

	// performs wa_star
	if(wa_star_params != NULL) {
		bool run_after = false;
		if(!g_params_list.empty())
			run_after = true;

		run_wa_star(wa_star_params, run_after);
	}
	return 1;
}

bool are_generating_landmarks(WA_Star_Parameters *wa_star_params) {
    for(int i = 0; i < g_params_list.size(); i++) {
        if(g_params_list[i]->heur == MRW_Parameters::LM)
            return true;
     }
    
    if(wa_star_params != NULL &&
            (wa_star_params->heuristics.count("LM") != 0 ||
                wa_star_params->pref_op_heuristics.count("LM") != 0)) {

        return true;
    }
    
    return false;
}

