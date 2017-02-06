#include "shared_mrw_parameters.h"

#include <iostream>

// default values for shared MRW parameters
#define DEFAULT_RES_TYPE BASIC
#define DEFAULT_POOL_ACT 50
#define DEFAULT_POOL_SIZE 50
#define DEFAULT_UCB_CONST 0.2

#define DEFAULT_RUN_ARAS false
#define DEFAULT_ARAS_TIME -1
#define DEFAULT_ARAS_MEM -1
#define DEFAULT_ARAS_REG true 
#define DEFAULT_FAST_ARAS false
#define DEFAULT_DOVETAIL false
#define DEFAULT_ADJUST_ONLINE false

#define DEFAULT_MRW_TIME_LIMIT -1

#define DEFAULT_NUM_THREADS 1

using namespace std;

Shared_MRW_Parameters::Shared_MRW_Parameters() {
    set_as_defaults();
}

void Shared_MRW_Parameters::set_as_defaults() {
    restart_type = DEFAULT_RES_TYPE;
    pool_size = DEFAULT_POOL_SIZE;
    act_level = DEFAULT_POOL_ACT;
    ucb_const = DEFAULT_UCB_CONST;
    run_aras = DEFAULT_RUN_ARAS;
    reg_aras = DEFAULT_ARAS_REG;
    fast_aras = DEFAULT_FAST_ARAS;
    aras_kb_limit = DEFAULT_ARAS_MEM;
    aras_time_limit = DEFAULT_ARAS_TIME;

    num_threads = DEFAULT_NUM_THREADS;

    dovetail = DEFAULT_DOVETAIL;

    adjust_online = false;

    mrw_time_limit = -1;
}

void Shared_MRW_Parameters::set_as_dummy() {
    restart_type = -1;
    pool_size = -1;
    act_level = -1;
    ucb_const = -1;
    run_aras = false;
    reg_aras = DEFAULT_ARAS_REG;  
    fast_aras = DEFAULT_FAST_ARAS;
    
    // -1 already implies infinity, I think
	aras_kb_limit = -2;
    aras_time_limit = -2;
    dovetail = false;

    num_threads = -1;

    mrw_time_limit = -2;
}

// TODO make more substantial
void Shared_MRW_Parameters::set_unset_params() {
    if(restart_type == -1)
        restart_type = DEFAULT_RES_TYPE;
    if(pool_size == -1 && restart_type == S_RESTART)
        pool_size = DEFAULT_POOL_SIZE;
    if(act_level == -1 && restart_type == S_RESTART)
        act_level = DEFAULT_POOL_ACT;
    if(ucb_const == -1 && !dovetail)
        ucb_const = DEFAULT_UCB_CONST;
    if(aras_kb_limit == -2)
        aras_kb_limit = DEFAULT_ARAS_MEM;
    if(aras_time_limit == -2)
        aras_time_limit = DEFAULT_ARAS_TIME;
    if(num_threads == -1)
    	num_threads = DEFAULT_NUM_THREADS;
    if(mrw_time_limit == -2)
    	mrw_time_limit = DEFAULT_MRW_TIME_LIMIT;

}

void Shared_MRW_Parameters::print_values() {
    cout << "\tRestart Type: ";
    if(restart_type == S_RESTART) {
        cout << "SMART" << endl;

        cout << "\t\tPool Size: " << pool_size << endl;
        cout << "\t\tActivation Level: " << act_level << endl;
    } else if(restart_type == BASIC) 
        cout << "BASIC" << endl;

    if(dovetail)
    	cout << "\tUsing dovetailing" << endl;
    else
    	cout << "\tUCB Constant Value: " << ucb_const << endl;

    cout << "\tUse Aras: ";
    if(run_aras) {
        cout << "true" << endl;
        cout << "\t\tregression graph: ";
	if(reg_aras){
	    cout << "true" << endl;
	}else
	    cout << "false" << endl;
        if(fast_aras)
	    cout << "\t\tRuns single iteration" << endl;
        cout << "\t\tAras Memory Limit: ";
        if(aras_kb_limit == -2)
            cout << "NONE" << endl;
        else
            cout << aras_kb_limit << endl;

        cout << "\t\tAras Time Limit: ";

        if(aras_time_limit == -2)
            cout << "NONE" << endl;
        else
            cout << aras_time_limit << endl;
    }else{
        cout << "false" << endl;
    }

    cout << "\tAdjusting online: ";
    if(adjust_online) {
    	cout << "true" << endl;
    } else {
    	cout << "false" << endl;
    }
    cout << "\tNum Threads: " << num_threads << endl;
    cout << "\tMRW Time Limit: ";
    if(mrw_time_limit < 0)
    	cout << "None" << endl;
    else
    	cout << mrw_time_limit << endl;
}

bool Shared_MRW_Parameters::read_from_string(string conf_string) {
    set_as_dummy();
    
    vector<string> tokens = split(conf_string, ' ');
    
    for(int i = 0; i < tokens.size(); i++) {
        
        // TODO handle other whitespace
        if(tokens[i].size() == 0)
            continue;
            
        if(tokens[i][0] != '-') {
            cerr << "MRW config arg " << tokens[i] << " called improperly" 
                    << endl;
            return false;
        }
    
        // boolean values
        if(tokens[i].compare("-run_aras") == 0) {

            if(run_aras) { // prevent duplicates
                cerr << "Can't set -run_aras multiple times " << endl;
                return false;
            }
            run_aras = true;
            
        } else if(tokens[i].compare("-dovetail") == 0) {
        	if(dovetail) { // prevent duplicates
        		cerr << "Can't set -dovetail multiple times " << endl;
        	    return false;
        	} else if(ucb_const != -1) {
        		cerr << "Can't enter -dovetail and a ucb_const " << endl;
        		return false;
        	}

        	dovetail = true;

        } else if(tokens[i].compare("-adjust_online") == 0) {
        	if(adjust_online) { // prevent duplicates
        		cerr << "Can't set -adjust_online multiple times " << endl;
        	    return false;
        	}

        	adjust_online = true;

        } else { // inputs that require a second argument
        
            string arg = tokens[i];
            
            do { // TODO handle other whitespace
                i++;
                if(i >= tokens.size()) {
                    cerr << arg << " has been entered without any value." << endl;
                    return false;
                }
            } while(tokens[i].size() == 0);

            // Handle float inputs
            if(arg.compare("-ucb_const") == 0) {
                if(ucb_const != -1) { // duplicates not allowed
                    cerr << "Can't set ucb constant value multiple times" << endl;
                    return false;
                } else if(dovetail) {
                	cerr << "Can't enter -dovetail and a ucb_const " << endl;
                	return false;
            	} else if(!string_to_float(tokens[i], ucb_const))
                    return false;
                    
                if(ucb_const < 0) { 
                    cerr << "-ucb_const value must be in the range [0,infty)" 
                        << endl;
                    return false;
                }
            // Handle int inputs
            } else if(arg.compare("-pool_size") == 0) {
                if(pool_size != -1) {
                    cerr << "Cannot set pool size more than once\n";
                    return false;
                } else if(!string_to_int(tokens[i], pool_size))
                    return false;
                    
                if(pool_size < 1) {
                    cerr << "pool size must be in the range [1,infty)" << endl;
                    return false;
                }
            } else if(arg.compare("-pool_act") == 0) {
                if(act_level != -1) {
                    cerr << "Cannot set pool activation level more than once\n";
                    return false;
                } else if(!string_to_int(tokens[i], act_level))
                    return false;
                    
                if(act_level < 1) {
                    cerr << "-pool_act must be in the range [1,infty)" << endl;
                    return false;
                }
            
            } else if(arg.compare("-aras_time") == 0) {
                if(aras_time_limit != -2) {
                    cerr << "Can't set aras time limit multiple times" << endl;
                    return false;
                } else if(!string_to_int(tokens[i], aras_time_limit))
                    return false;
                    
                if(aras_time_limit < 1 && aras_time_limit != -1) {
                    cerr << "Aras time limit must be in the range " <<
                        "{-1}U[1,infty), where -1 implies no limit" << endl;
                    return false;
                }
            } else if(arg.compare("-aras_mem") == 0) {
                if(aras_kb_limit != -2) {
                    cerr << "Can't set aras time limit multiple times" << endl;
                    return false;
                } else if(!string_to_int(tokens[i], aras_kb_limit))
                    return false;
                    
                if(aras_kb_limit < 1 && aras_kb_limit != -1) {
                    cerr << "Aras byte limit must be in the range " <<
                        "{-1}U[1,infty), where -1 implies no limit" << endl;
                    return false;
                }
            } else if(arg.compare("-mrw_time_limit") == 0) {
                if(mrw_time_limit != -2) {
                    cerr << "Can't set mrw time limit multiple times" << endl;
                    return false;
                } else if(!string_to_int(tokens[i], mrw_time_limit))
                    return false;

                if(mrw_time_limit < 1 && mrw_time_limit != -1) {
                    cerr << "MRW time limit must be in the range " <<
                        "{-1}U[1,infty), where -1 implies no limit" << endl;
                    return false;
                }
            } else if(arg.compare("-num_threads") == 0) {
            	if(num_threads != -1) {
            		cerr << "Can't set num threads multiple times" << endl;
            		return false;
            	} else if(!string_to_int(tokens[i], num_threads))
                    return false;

            	if(num_threads < 1) {
            		cerr << "Number of threads must be positive" << endl;
            		return false;
            	}

            // now consider inputs with a small finite number of string options 
            } else if(arg.compare("-res_type") == 0) {
               if(restart_type != -1) {
                    cerr << "Cannot set restart type more than once\n";
                    return false; 
                } else if(tokens[i].compare("BASIC") == 0)
                    restart_type = Shared_MRW_Parameters::BASIC;
                else if(tokens[i].compare("SMART") == 0)
                    restart_type = Shared_MRW_Parameters::S_RESTART;
                else {
                    cerr << "Invalid restart type entered" << endl;
                    return false;
                }
            // invalid input
            } else {
                cerr << "Invalid shared MRW parameter arg of "
                    << arg << " entered" << endl;
                return false;
            }
            
        }
    }    
    
    if(!check_for_conflicts()) {
    	cerr << "Error found in Shared MRW configuration." << endl;
    	return false;
    }

    set_unset_params();
    return true;
}

void Shared_MRW_Parameters::print_help_info() {
    cerr << "Including -mrw_shared \"CONF\" sets up a run of Arvand"
    		<< endl;
    cerr << "CONF can include the following options" << endl;
    cerr << "\t-res_type BASIC|SMART : sets restart type. BASIC by default" 
        << endl;
    cerr << "\t-pool_act n : changes activation level to n for n in [1,infty)." 
        << "\n\t\t" << DEFAULT_POOL_ACT << " by default" << endl;
    cerr << "\t-pool_size n : changes pool size to n for n in [1,infty). " 
        << DEFAULT_POOL_ACT << " by default" << endl;
    cerr << "\t-ucb_const n : ucb constant of parameter learner " <<
        " is changed to n for\n\t\tn in [0, infty). " <<
        DEFAULT_UCB_CONST << " by default" << endl;
    cerr << "\t\tAlternatively enter -dovetail for dovetailing" << endl;
    cerr << "\t-run_aras : uses aras to improve plans found" << endl;
    cerr << "\t-aras_mem n : limits aras memory usage to n bytes. " <<
        "\n\t\tn in {-1}U[1, infty). -1 means no limit (is default value)"
    		<< endl;
    cerr << "\t-aras_time n : limits aras time limit to n seconds. " <<
        "\n\t\tn in {-1}U[1, infty). -1 means no limit (is default value)"
    		<< endl;
    cerr << "\t-adjust_online: adjusts restart frequency over time" << endl;
    cerr << "\t-num_threads n : sets number of mrw threads to run to n" << endl;
    cerr << "\t-mrw_time_limit i : sets time limit for MRW to i seconds"
    		<< "\n\t\tn in {-1}U[1, infty). -1 means no limit (is default value)"
    		<< endl;
}

bool Shared_MRW_Parameters::check_for_conflicts() {

	// ensure smart restarts have been set properly
	if(restart_type != Shared_MRW_Parameters::S_RESTART) {
		if(act_level != -1) {
			cerr << "Restart activation level set without smart restarting\n";
			return false;
		} else if(pool_size != -1) {
			cerr << "Pool size set without smart restarting\n";
			return false;
		}
	}

	if(!run_aras) {
		if(aras_kb_limit >= -1) {
			cerr << "Aras byte limit set without run aras set" << endl;
			return false;
		}
		if(aras_time_limit >= -1) {
			cerr << "Aras time limit set without run aras set" << endl;
			return false;
		}
	}

	if(mrw_time_limit >= 0 && num_threads > 1) {
		cerr << "Time limits for mrw not currently enabled when using more than one thread" << endl;
		return false;
	}

	return true;
}
