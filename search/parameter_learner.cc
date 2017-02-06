#include "parameter_learner.h"
#include "math.h"
#include "globals.h"

UCB::UCB(float ucb_const, bool adjusting, MTRand_int32 *r) : c(ucb_const),
		adjust_online(adjusting), rand_gen(r) {
	values.resize(g_params_list.size());
	n.resize(g_params_list.size());
	total_n = 0;
	initial_num_walks = 100;
	initial_max_steps = 1;

	num_walk_ub = 2000;
	max_steps_ub = 7;
}

int UCB::get_config(){

	int config_id = 0;
	bool unused = false;

    // only one config, so just return it
    if(n.size() != 1) {

    	float max = -1;
    	vector<int> arg_max_list;
    	vector<int> not_tried;
    	for (int i = 0; i < n.size(); ++i) {
    		if(n[i] == 0){
    			not_tried.push_back(i);
    		}else{

    			float ucb_val;

    			if(c >= 0)
    				ucb_val = values[i] + c * sqrt(2*log(total_n)/n[i]);
    			else // if dovetailing, give highest reward to least used arms
    				ucb_val = total_n - n[i];

    			if(max < ucb_val){
    				max = ucb_val;
    				arg_max_list.clear();
    				arg_max_list.push_back(i);
    			}else if (max == ucb_val){
    				arg_max_list.push_back(i);
    			}
    		}
    	}

    	if(not_tried.size() != 0){
    		// at least one of the configurations has not been tried
    		config_id = not_tried[rand_gen->get_32bit_int() % not_tried.size()];
    		unused = true;
    	} else {
    		assert(!arg_max_list.empty());

    		// if one single best value
    		if(arg_max_list.size() == 1)
    			config_id = arg_max_list[0];
    		else
    			config_id = arg_max_list[rand_gen->get_32bit_int() %
    			                         arg_max_list.size()];
    	}
    } else if(n[config_id] == 0) { // one config that is unused
    	unused = true;
    }

    if(adjust_online) {
    	if(unused){
    		g_params_list[config_id]->num_walk = initial_num_walks;
    		g_params_list[config_id]->max_steps = initial_max_steps;
    	} else{

    		g_params_list[config_id]->num_walk = min(2*g_params_list[config_id]->num_walk,
    				num_walk_ub);
    		g_params_list[config_id]->max_steps = min(g_params_list[config_id]->max_steps + 1,
    				max_steps_ub);
    	}
    }

	// entering temporary reward of 0 to encourage exploration
	// virtual loss idea from the parallel UCT
	// Note, is equivalent to no virtual loss in single-core version
	n[config_id]++;
	values[config_id] -= values[config_id]/float(n[config_id]);
	total_n++;

	return config_id;
}

void UCB::update_value(int i, int h, int upper_bound,
		const string &thread_name){
	// First the heuristic value is mapped to the range [0 1]
	// then it is used to update the average value
	h = min(upper_bound, h);

	cout << thread_name;
	for (int var = 0; var < n.size(); ++var) {
		string str;
		if(g_params_list[var]->walk_type == 0)
			str = "PURE";
		else if(g_params_list[var]->walk_type == 1)
			str = "MDA";
		else if(g_params_list[var]->walk_type == 2)
			str = "MHA";
		cout << str << "-" << g_params_list[var]->length_walk
				<< ":"<< n[var] << " ";
	}
	cout << endl;
	float reward = 1;
	if(upper_bound != 0)
	    reward = (upper_bound - h)/float(upper_bound);
	values[i] += reward/float(n[i]);
}




