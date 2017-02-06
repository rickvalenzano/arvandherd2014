#include "wa_star_params.h"

#include <iostream>
#include <fstream>

// default lama parameters
#define WA_PARAMS_DEFAULT_PREF_REWARD 1000
#define WA_PARAMS_DEFAULT_MEM_LIMIT -1

#define WA_PARAMS_DEFAULT_ARAS_MEM -1
#define WA_PARAMS_DEFAULT_ARAS_TIME -1

#define WA_PARAMS_DEFAULT_EPSILON 0
#define WA_PARAMS_DEFAULT_INIT_EXP_LIMIT -1
#define WA_PARAMS_DEFAULT_EXP_FACTOR 2

// TODO need to offer command line parsing for these
#define WA_DEFAULT_ARAS_REG true
#define WA_DEFAULT_FAST_ARAS false

#define WA_DEFAULT_IGNORE false
#define WA_DEFAULT_LOOP_WEIGHTS false
#define WA_PARAMS_DEFAULT_BOUND_TYPE WA_Star_Parameters::FULL

using namespace std;

WA_Star_Parameters::WA_Star_Parameters() {    
    set_to_dummy();
    set_unset_params();

    //TODO see the to do above
    reg_aras = WA_DEFAULT_ARAS_REG;
    fast_aras = WA_DEFAULT_FAST_ARAS;
    ignore_costs = WA_DEFAULT_IGNORE;
    loop_weights = WA_DEFAULT_LOOP_WEIGHTS;
}

void WA_Star_Parameters::print_values() {

	if(weights.size() == 1) {
		if(weights[0] == -1) {
			cout << "\t\tGBFS with deferred heuristic evaluation" << endl;
		} else {
			cout << "\t\tWA* with deferred heuristic evaluation and weight of " <<
					weights[0] << endl;
		}
	} else {
		cout  << "\tIterative WA* with deferred heuristic " <<
				"evaluation. Weights used in the following order: "
				<< endl;
		for(unsigned i = 0; i < weights.size(); i++) {
			if(weights[i] == -1)
				cout << "\t\tGBFS" << endl;
			else
				cout << "\t\t" << weights[i] << endl;
		}
	}
    cout << "\tUsing the following heuristics: " << endl;
    
    set<string>::iterator it;
    for(it = heuristics.begin(); it != heuristics.end(); it++)
        cout << "\t\t" << *it << endl;
    
    cout << "\tUsing the following heuristics " <<
            "for generating preferred operators:" << endl;
            
    for(it = pref_op_heuristics.begin(); it != pref_op_heuristics.end(); it++)
        cout << "\t\t" << *it << endl;
        
    cout << "\tRandomizing Generated States: ";
    if(rand_open)
        cout << "true" << endl;
    else
        cout << "false" << endl;

    cout << "\tIgnore action costs: ";
    if(ignore_costs)
    	cout << "true" << endl;
    else
    	cout << "false" << endl;

    cout << "\tIndefinitely loop weights: ";
    if(loop_weights)
    	cout << "true" << endl;
    else
    	cout << "false" << endl;

    cout << "\tPreference Priority Reward: " << pref_reward << endl;

    cout << "\tMemory Limit: " << kb_limit << endl;

    cout << "\tEpsilon: " << epsilon << endl;

    cout << "\tInit Node Expansion Limit: " << init_node_limit << endl;
    cout << "\tNode Limit Factor: " << node_limit_factor << endl;

    cout << "\tUse Aras: ";
    if (run_aras) {
		cout << "true" << endl;
		cout << "\t\tregression graph: ";
		if (reg_aras) {
			cout << "true" << endl;
		} else
			cout << "false" << endl;
		if (fast_aras)
			cout << "\t\tRuns single iteration" << endl;
		cout << "\t\tAras Memory Limit: ";
		if (aras_kb_limit == -2)
			cout << "NONE" << endl;
		else
			cout << aras_kb_limit << endl;

		cout << "\t\tAras Time Limit: ";

		if (aras_time_limit == -2)
			cout << "NONE" << endl;
		else
			cout << aras_time_limit << endl;
	} else {
		cout << "false" << endl;
	}

    cout << "\tBounding Type: ";
    if(bounding_type == FULL)
    	cout << "Full" << endl;
    else if(bounding_type == NONE)
    	cout << "No bounding" << endl;
    else if(bounding_type == WA)
    	cout << "WA* Solutions only" << endl;
    else if(bounding_type == DAS)
    	cout << "DAS" << endl;

}

bool WA_Star_Parameters::read_from_string(string conf_string) {
    set_to_dummy();
    
    vector<string> tokens = split(conf_string, ' ');
    
    for(int i = 0; i < tokens.size(); i++) {
    
        // TODO handle other whitespace
        if(tokens[i].size() == 0)
            continue;
            
        if(tokens[i][0] != '-') {
            cerr << "WA* config arg " << tokens[i] << " called improperly" 
                    << endl;
            return false;
        }
        
        // boolean conditions first
        if(parse_extra_bool_conditions(tokens[i])) {
            // if succeed, do nothing
        } else { // inputs that require a second argument
        
            string arg = tokens[i];
            
            do { // TODO handle other whitespace
                i++;
                
                if(i >= tokens.size()) {
                    cerr << arg << " has been entered without any value." << endl;
                    return false;
                }
            } while(tokens[i].size() == 0);              
            
            if(arg.compare("-heur") == 0) {
                if(!parse_heuristic(tokens[i]))
                    return false;       
            } else if(arg.compare("-weight_list") == 0) {
                if(!parse_weight_list(tokens[i]))
                    return false;
            } else if(arg.compare("-pref") == 0) {
                if(!parse_preferred(tokens[i]))
                    return false;
            } else if(arg.compare("-mem_limit") == 0) {
                if(kb_limit != -2) {
                    cerr << "Cannot enter WA* kilobyte limit multiple times"
                        << endl;
                    return false;
                } else if(!string_to_double(tokens[i], kb_limit))
                    return false;
            
                if(kb_limit < 1 && kb_limit != -1) {
                    cerr << "-mem_limit value must be in the range " <<
                        "{-1}U[1,infty), where -1 implies no limit" << endl; 
                    return false;
                }
            }  else if(arg.compare("-aras_mem") == 0) {
                if(aras_kb_limit != -2) {
                    cerr << "Cannot enter WA* Aras byte limit multiple times"
                        << endl;
                    return false;
                } else if(!string_to_int(tokens[i], aras_kb_limit))
                    return false;

                if(aras_kb_limit < 1 && aras_kb_limit != -1) {
                    cerr << "-aras_mem value must be in the range " <<
                        "{-1}U[1,infty), where -1 implies no limit" << endl;
                    return false;
                }
            } else if(arg.compare("-aras_time") == 0) {
                if(aras_time_limit != -2) {
                    cerr << "Cannot enter WA* Aras time limit multiple times"
                        << endl;
                    return false;
                } else if(!string_to_int(tokens[i], aras_time_limit))
                    return false;

                if(aras_time_limit < 1 && aras_time_limit != -1) {
                    cerr << "-aras_time value must be in the range " <<
                        "{-1}U[1,infty), where -1 implies no limit" << endl;
                    return false;
                }
            } else if(!parse_extra_2_arg_conditions(arg, tokens[i])) {
                cerr << "Invalid lama config option " << arg << endl;
                return false;
            }
        }
    }
    
    if(!check_for_conflicts()) {
    	cerr << "Error found in the following WA* configuration:" << endl;
    	cerr << conf_string << endl;
        return false;
    }

    set_unset_params();
    
    if((pref_op_heuristics.count("LAMA_FF") && 
            pref_op_heuristics.count("LAMA_FF_S")) ||
       (pref_op_heuristics.count("LAMA_FF_S") && 
            pref_op_heuristics.count("LAMA_FF_C")) ||
       (pref_op_heuristics.count("LAMA_FF") && 
            pref_op_heuristics.count("LAMA_FF_C"))) {

        cerr << "WARNING: Have entered duplicate preferred operators" << endl;
    }
    
    return true;
}

bool WA_Star_Parameters::parse_heuristic(string str_heur) {

    if(str_heur.compare("LAMA_FF") != 0 && 
       str_heur.compare("LAMA_FF_S") != 0 &&
       str_heur.compare("LAMA_FF_C") != 0 &&
       str_heur.compare("FD_FF") != 0 &&
       str_heur.compare("LM") != 0 &&
       str_heur.compare("LM") != 0 &&
       str_heur.compare("BLIND") != 0 &&
       str_heur.compare("GOAL_COUNT") != 0 &&
       str_heur.compare("FF_SWITCH") != 0 &&
       str_heur.compare("FD_SWITCH") != 0) {
    
        cerr << str_heur << " is an invalid heuristic" << endl;
        return false;
    }
    pair<set<string>::iterator,bool> ret;

    ret = heuristics.insert(str_heur);
    
    if(!ret.second) {
        cerr << str_heur << " entered twice for the same config as " <<
             " a heuristic" << endl;
        return false;
    }
    return true;
}

bool WA_Star_Parameters::parse_preferred(string str_pref) {

    if(str_pref.compare("LAMA_FF") != 0 && 
       str_pref.compare("LAMA_FF_S") != 0 &&
       str_pref.compare("LAMA_FF_C") != 0 &&
       str_pref.compare("FD_FF") != 0 &&
       str_pref.compare("LM") != 0 &&
       str_pref.compare("FF_SWITCH") != 0 &&
       str_pref.compare("FD_SWITCH") != 0) {

        cerr << str_pref << " is an invalid preferred operator heuristic" << 
            endl;
        return false;
    }
    pair<set<string>::iterator,bool> ret;

    ret = pref_op_heuristics.insert(str_pref);
    
    if(!ret.second) {
        cerr << str_pref << " entered twice for the same config as " <<
             " a heuristic" << endl;
        return false;
    }
    return true;
}

bool WA_Star_Parameters::parse_weight_list(string str_list) {
    if(str_list[0] != '[' || str_list[str_list.size()-1] != ']') {
        cerr << "Bad List formatting" << endl;
        return false;
    } 
    
    // split string by commas, eliminating square brackets first
    vector<string> tokens = split(str_list.substr(1, str_list.size()-2), ',');

    // want best-first search
    if(tokens.size() == 0) {
        return true;
    }

    int new_weight;
    for(int i = 0; i < tokens.size(); i++) {
        if(!string_to_int(tokens[i], new_weight))
            return false;
            
        if(new_weight != -1 && new_weight < 0) {
            cerr << "Invalid list element \"" << tokens[i] << " entered." << endl;
            return false;
        }

        weights.push_back(new_weight);
    }
    return true;
}

void WA_Star_Parameters::set_to_dummy() {

    heuristics.clear();
    pref_op_heuristics.clear();
    
    weights.clear();

    pref_reward = -2;
    kb_limit = -2;
    epsilon = -2;
    
    init_node_limit = -2;
    node_limit_factor = -2;

    is_pref_reward_set = false;
    is_eps_set = false;
    is_init_exp_limit_set = false;
    is_exp_factor_set = false;

    rand_open = false;
    run_aras = false;
    ignore_costs = false;

    aras_kb_limit = -2;
    aras_time_limit = -2;

    bounding_type = -1;
}

void WA_Star_Parameters::set_unset_params() {
    if(!is_pref_reward_set)
        pref_reward = WA_PARAMS_DEFAULT_PREF_REWARD;
    if(kb_limit == -2)
        kb_limit = WA_PARAMS_DEFAULT_MEM_LIMIT;
    if(aras_kb_limit == -2)
    	aras_kb_limit = WA_PARAMS_DEFAULT_ARAS_MEM;
    if(aras_time_limit == -2)
    	aras_time_limit = WA_PARAMS_DEFAULT_ARAS_TIME;
    if(!is_eps_set)
    	epsilon = WA_PARAMS_DEFAULT_EPSILON;
    if(!is_init_exp_limit_set)
    	init_node_limit = WA_PARAMS_DEFAULT_INIT_EXP_LIMIT;
    if(!is_exp_factor_set)
    	node_limit_factor = WA_PARAMS_DEFAULT_EXP_FACTOR;

    if(weights.size() == 0) {
    	weights.push_back(-1);
    }

    if(bounding_type == -1)
    	bounding_type = WA_PARAMS_DEFAULT_BOUND_TYPE;
}

bool WA_Star_Parameters::parse_extra_2_arg_conditions(
                            const std::string arg, const std::string value) {
    
    if(arg.compare("-p_reward") == 0) {
    
        if(is_pref_reward_set) {
            cerr << "Cannot enter -p_reward multiple times" << endl;
            return false;
        } else if(!string_to_int(value, pref_reward))
            return false;
            
        is_pref_reward_set = true;
        return true;
    } else if(arg.compare("-epsilon") == 0) {
        if(is_eps_set) {
            cerr << "Cannot enter WA* epsilon value multiple times"
                << endl;
            return false;
        } else if(!string_to_double(value, epsilon))
            return false;

        if(epsilon < 0.0 || epsilon > 1.0) {
        	cerr << "-epsilon value must be in the range [0,1]" << endl;
        	return false;
        }

        is_eps_set = true;
        return true;
    } else if(arg.compare("-init_exp_limit") == 0) {

        if(is_init_exp_limit_set) {
            cerr << "Cannot enter -init_exp_limit multiple times" << endl;
            return false;
        } else if(!string_to_int(value, init_node_limit))
            return false;

        if(init_node_limit <= 0) {
        	cerr << "-init_exp_limit value must be in the range (0,infty)" << endl;
        	return false;
        }

        is_init_exp_limit_set = true;
        return true;
    } else if(arg.compare("-exp_limit_factor") == 0) {

        if(is_exp_factor_set) {
            cerr << "Cannot enter -exp_limit_factor multiple times" << endl;
            return false;
        } else if(!string_to_double(value, node_limit_factor))
            return false;

        if(node_limit_factor < 1) {
        	cerr << "-exp_limit_factor value must be in the range [1,infty)" << endl;
        	return false;
        }

        is_exp_factor_set = true;
        return true;
    } else if(arg.compare("-bound_type") == 0) {
    	if(bounding_type != -1) {
    		cerr << "Cannot set restart type more than once\n";
    		return false;
    	} else if(value.compare("FULL") == 0)
    		bounding_type = WA_Star_Parameters::FULL;
    	else if(value.compare("NONE") == 0)
    		bounding_type = WA_Star_Parameters::NONE;
    	else if(value.compare("WA") == 0)
    		bounding_type = WA_Star_Parameters::WA;
    	else if(value.compare("DAS") == 0)
    	    bounding_type = WA_Star_Parameters::DAS;
    	else {
    		cerr << "Invalid bounding type entered" << endl;
    		return false;
    	}
    	return true;
    }
    return false;                                                
}

bool WA_Star_Parameters::parse_extra_bool_conditions(const string arg){
    
    if(arg.compare("-rand_open") == 0) {
        if(rand_open) {
            cerr << "Cannot enter -rand_open multiple times" << endl;
            return false;
        }
        
        rand_open = true;
        return true;
    } else if(arg.compare("-run_aras") == 0) {
    	if(run_aras) {
    		cerr << "Cannot enter -run_aras multiple times" << endl;
    		return false;
    	}

    	run_aras = true;
    	return true;
    } else if(arg.compare("-ignore_costs") == 0) {
    	if(ignore_costs) {
    		cerr << "Cannot enter -ignore_costs multiple times" << endl;
    		return false;
    	}

    	ignore_costs = true;
    	return true;
    } else if(arg.compare("-loop_weights") == 0) {
    	if(loop_weights) {
    		cerr << "Cannot enter -loop_weights multiple times" << endl;
    		return false;
    	}

    	loop_weights = true;
    	return true;
    }
    return false;                                            
}
       
void WA_Star_Parameters::print_help_info() {
    cerr << "Including -wa_conf \"CONF\" sets up a run of" <<
        " WA* with delayed heuristic evaluation" << endl;
    cerr << "CONF can include the following options" << endl;
    cerr << "\t-heur X : X is added to the list of heuristics (see below)" << endl;
    cerr << "\t-pref X : X is added to the list of heuristics used for " 
        << "preferred\n\t\toperator generation (see below)" << endl; 
    cerr << "\t-weight_list [w1,w2,...,wn] : wi's are weights to be" <<
        " used in order" << endl;
    cerr << "\t-rand_open : enables randomization of generated nodes" << endl;
    cerr << "\t-p_reward n : sets the preference priority reward to n. " <<     
        WA_PARAMS_DEFAULT_PREF_REWARD << " by default" << endl;
    cerr << "\t-mem_limit n : limits the memory usage to only n kilobytes. None " <<
        "by default" << endl;
    cerr << "\t-run_aras : uses aras to improve plans found" << endl;
    cerr << "\t-aras_mem n : limits aras memory usage to n bytes. " <<
    		"\n\t\tn in {-1}U[1, infty). -1 means no limit (is default value)"
    		<< endl;
	cerr << "\t-aras_time n : limits aras time limit to n seconds. "
			<< "\n\t\tn in {-1}U[1, infty). -1 means no limit (is default value)"
			<< endl;
	cerr <<" \t-ignore_costs : ignores action costs in g-values" << endl;
	cerr << "\t-epsilon e: set value of epsilon to e for epsilon-greedy (0.0 is the default)" << endl;
	cerr << "\t-loop_weights: loop over the weight list indefinitely" << endl;
	cerr << "\t-init_exp_limit n: sets the initial expansion limit for a weight to be n"
			<< "\n\t\tn in {-1}U[1, infty). -1 means no limit (is default value)"
			<< endl;
	cerr << "\t-exp_limit_factor f: sets the factor by which the limit increases to f"
			<< "\n\t\tf must be in the range from [1, infty)"
			<< endl;
}                                     

bool WA_Star_Parameters::check_for_conflicts() {

	bool use_ff_switch = heuristics.count("FF_SWITCH") != 0;
	bool use_ff_switch_prefs = pref_op_heuristics.count("FF_SWITCH") != 0;

	if(use_ff_switch_prefs && !use_ff_switch) {
		cerr << "Can't use FF_SWITCH for only preferred operators." << endl;
		return false;
	}

	bool use_fd_switch = heuristics.count("FD_SWITCH") != 0;
	bool use_fd_switch_prefs = pref_op_heuristics.count("FD_SWITCH") != 0;

	if(use_fd_switch_prefs && !use_fd_switch) {
		cerr << "Can't use FD_SWITCH for only preferred operators." << endl;
		return false;
	}

	// no lama heuristics have been entered
	if(heuristics.empty()) {

		cerr << "Can't give a WA* config without any heuristics" << endl;
		return false;
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
	return true;
}
