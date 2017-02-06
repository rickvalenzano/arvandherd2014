#include "axioms.h"

#include "wa_runner.h"
#include "delayed_wa_star.h"

#include "landmarks_graph.h"
#include "string.h"

#include "ff_heuristic.h"
#include "fd_ff_heuristic.h"
#include "lama_ffc_heuristic.h"
#include "lama_ffs_heuristic.h"
#include "blind_search_heuristic.h"
#include "landmarks_count_heuristic.h"
#include "goal_count_heuristic.h"
#include "mrw.h"
#include "mrw_runner.h"
#include "memory_utils.h"

// heuristics used for lama
// keep them around anytime version of LAMA
LandmarksCountHeuristic *lama_lm_heur = NULL;
FFHeuristic *lama_ff_heur = NULL;
FDFFHeuristic *lama_fd_ff_heur = NULL;
LAMA_FFC_Heuristic *lama_ffc_heur = NULL;
LAMA_FFS_Heuristic *lama_ffs_heur = NULL;

// adds the heuristics to a best first search engine
void add_heuristics(WA_Star_Parameters *wa_star_params, DelayedWAStar* engine,
		AxiomEvaluator *axiom_eval);

void run_wa_star(WA_Star_Parameters *wa_star_params, bool run_mrw_after) {

	string name = "";
	if(g_mrw_shared != NULL) {
		name = "$WA: ";
	}

	AxiomEvaluator *axiom_eval = new AxiomEvaluator;
    if (g_lgraph != NULL && g_lgraph->number_of_landmarks() == 0) {
        wa_star_params->heuristics.erase("LM");
        wa_star_params->pref_op_heuristics.erase("LM");
        
        cout << "Landmarks removed from WA* heuristics list" << endl;
            
        // if no other heuristics are set as to be used
        if(wa_star_params->heuristics.empty()) {
            
            cout << "LAMA_FF_S added to LAMA heuristics list." << endl;
            // use lama_ffs
            wa_star_params->heuristics.insert("LAMA_FF_S");
        }
    }
    
    int iteration_no = 0;
    bool solution_found = false;
	
	int wastar_weight = -1;

    MTRand_int32 rand_gen(get_current_seed(77));

    int current_node_limit = wa_star_params->init_node_limit;
    int count_out_of_memory = 0;
    int first_iter_with_solution = -1;
    int current_bound = -1;
	
	do {
		// if have already found a solution and not iterative, don't continue looking
		if(!g_iterative && g_best_sol_cost >= 0)
			break;

        wastar_weight = wa_star_params->weights[iteration_no % wa_star_params->weights.size()];

        if(iteration_no < 0) // if overflow has happened, switch to MRWs
        	break;
        if(current_node_limit > 0 && wa_star_params->node_limit_factor > 1 &&
        		iteration_no > 0 && iteration_no % wa_star_params->weights.size() == 0) {

        	// stick with same bound for one pass through once found a solution
        	if(first_iter_with_solution == -1 || iteration_no > first_iter_with_solution + wa_star_params->weights.size())
        		current_node_limit = ((double) current_node_limit)*wa_star_params->node_limit_factor;
        }

        cout << endl << endl << "Search iteration " << iteration_no << endl;
        cout << "Current Weight: " << wastar_weight << endl;
        cout << "Current Node Limit: " << current_node_limit << endl;

		// Initialize search engine and heuristics 
		// (this is cheap and we want to vary search type
		// and heuristics, so we initialize freshly in each iteration)
		DelayedWAStar* engine = new DelayedWAStar(axiom_eval,
				iteration_no, &rand_gen, wastar_weight, name,
				wa_star_params->ignore_costs,
				wa_star_params->rand_open, wa_star_params->epsilon);
           
        engine->set_priority_reward(wa_star_params->pref_reward);
        engine->set_byte_limit(wa_star_params->kb_limit);
        engine->set_expansion_limit(current_node_limit);
        
        cout << "adding heuristics ..."<< endl;
		add_heuristics(wa_star_params, engine, axiom_eval);

        //print_peak_memory();

        if(wa_star_params->bounding_type == WA_Star_Parameters::NONE) {
        	engine->set_use_local_bound();
        	engine->set_local_bound(-1);
        } else if(wa_star_params->bounding_type == WA_Star_Parameters::WA ||
        		wa_star_params->bounding_type == WA_Star_Parameters::DAS) {
        	engine->set_use_local_bound();

        	if(wa_star_params->bounding_type == WA_Star_Parameters::DAS &&
        			iteration_no % wa_star_params->weights.size() == 0)
        		current_bound = -1;
        	engine->set_local_bound(current_bound);
        	cout << "Current Bound: " << current_bound << endl;
        }

		Timer search_timer;
		int status = engine->search();
		search_timer.stop();

		// if solution is found
		if(engine->found_solution()) {
			if(first_iter_with_solution < 0)
				first_iter_with_solution = iteration_no;

        	Path current_plan = engine->get_plan();
            int last_plan_cost = save_plan(current_plan, g_output_filename, name, false);

            if(current_bound == -1 || last_plan_cost < current_bound)
            	current_bound = last_plan_cost;

            // run aras if need be
            if(wa_star_params->run_aras) {
				PlanBooster *booster = new PlanBooster(axiom_eval,
						wa_star_params->aras_kb_limit,
						wa_star_params->aras_time_limit, &rand_gen,
						name, wa_star_params->fast_aras);
				booster->any_time_neighborhood_search_star(current_plan,
							wa_star_params->reg_aras, true);
				delete booster;
            }
        }
		engine->statistics();

        cout << "Estimated Memory Usage: " << engine->memory_estimate()/1000 << " kb." << endl;
        double vm, rss;
        process_mem_usage(vm, rss);
        cout << "Total Memory Usage: " << vm << " kb." << endl;

		cout << "Search time: " << search_timer << endl;
		cout << "Total time: " << g_timer << endl;
		solution_found |= engine->found_solution();

        if(status == SearchEngine::OUT_OF_MEMORY) {
        	count_out_of_memory++;
        	cout << "Ran out of memory. Count: " << count_out_of_memory << endl;

        	// empty the closed list if had a reasonable number of out of memories
        	// TODO Make command line argument
        	if(count_out_of_memory %  wa_star_params->weights.size() == 0) {
        		cout << "Emptying Closed List" << endl;
        		g_closed_list.clear();
        	}

        	// default to random walks if ran out of memory a lot
        	// TODO Make command line argument
        	if(run_mrw_after && count_out_of_memory >= wa_star_params->weights.size()*2) {
        		cout << "Hit memory limit too many times. Reverting to MRW" << endl;
        		break;
        	}
        }

        iteration_no++;
        delete engine;
	}
	while(wa_star_params->loop_weights || iteration_no < wa_star_params->weights.size());

	// if should run mrw afterwards
	if(run_mrw_after && (g_iterative || g_best_sol_cost == -1)) {
		assert(g_mrw_shared != NULL);

		struct mrw_thread_data mrw_data;
		mrw_data.seed = get_current_seed(1992);
		mrw_data.name = name;

		// should only get here if there is only one thread
		if(g_mrw_shared->num_threads == 1) {
			g_mrw_shared->mrw_time_limit = -1;
		}
		run_mrw_thread((void *) &mrw_data);
	}
}

void add_heuristics(WA_Star_Parameters *wa_star_params, DelayedWAStar* engine,
		AxiomEvaluator *axiom_eval){
	
	bool use_lm = wa_star_params->heuristics.count("LM") != 0;
    bool use_lm_prefs = wa_star_params->pref_op_heuristics.count("LM") != 0;

    bool use_ff_switch = wa_star_params->heuristics.count("FF_SWITCH") != 0;
    bool use_ff_switch_prefs = wa_star_params->pref_op_heuristics.count("FF_SWITCH") != 0;

    bool use_fd_switch = wa_star_params->heuristics.count("FD_SWITCH") != 0;
    bool use_fd_switch_prefs = wa_star_params->pref_op_heuristics.count("FD_SWITCH") != 0;

    bool use_ff = wa_star_params->heuristics.count("LAMA_FF") != 0;
    bool use_ff_prefs = wa_star_params->pref_op_heuristics.count("LAMA_FF") != 0;	

    bool use_ffs = wa_star_params->heuristics.count("LAMA_FF_S") != 0;
	bool use_ffs_prefs = wa_star_params->pref_op_heuristics.count("LAMA_FF_S") != 0;
    
    bool use_ffc = wa_star_params->heuristics.count("LAMA_FF_C") != 0;
	bool use_ffc_prefs = wa_star_params->pref_op_heuristics.count("LAMA_FF_C") != 0;
		
	bool use_fd_ff = wa_star_params->heuristics.count("FD_FF") != 0;
	bool use_fd_ff_prefs = wa_star_params->pref_op_heuristics.count("FD_FF") != 0;	
	
	bool use_blind = wa_star_params->heuristics.count("BLIND") != 0;
	
	bool use_goal_count = wa_star_params->heuristics.count("GOAL_COUNT") != 0;

	/** 
    LM-count should always be added before lama_ff_heur if lm-count is using
	preferred operators. This is because sometimes the value calculated during
	the preferred operators segment can be cached and no new computation is
	needed when querying lama_ff for a heuristic. This can greatly speed up
	the search
	**/
	if(use_lm || use_lm_prefs) {
		
		if(lama_ff_heur == NULL)
            lama_ff_heur = new FFHeuristic;
            
        if(lama_lm_heur == NULL)
            lama_lm_heur = new LandmarksCountHeuristic(*g_lgraph, use_lm_prefs, 
                lama_ff_heur);

		engine->add_heuristic(lama_lm_heur, use_lm, use_lm_prefs);
	}

    /**
    Lama_FF should also be called before any of the other ones built on LAMA_FF
    so that the values can simply be reused as necessary.
    **/  
    if(use_ff || use_ff_prefs) {
        
        if(lama_ff_heur == NULL)
            lama_ff_heur = new FFHeuristic;
		
        engine->add_heuristic(lama_ff_heur, use_ff, use_ff_prefs);
	}
    
    if(use_ffs || use_ffs_prefs) {
         
        if(lama_ff_heur == NULL)
            lama_ff_heur = new FFHeuristic;
	    
	    if(lama_ffs_heur == NULL)
            lama_ffs_heur = new LAMA_FFS_Heuristic(lama_ff_heur);
            
        engine->add_heuristic(lama_ffs_heur, use_ffs, use_ffs_prefs);
	}

    if(use_ffc || use_ffc_prefs) {

        if(lama_ff_heur == NULL)
            lama_ff_heur = new FFHeuristic;

        if(lama_ffc_heur == NULL)
            lama_ffc_heur = new LAMA_FFC_Heuristic(lama_ff_heur);
		
		engine->add_heuristic(lama_ffc_heur, use_ffc, use_ffc_prefs);
	}

	if(use_ff_switch) {
		if(lama_ff_heur == NULL)
			lama_ff_heur = new FFHeuristic;
		if(lama_ffs_heur == NULL)
			lama_ffs_heur = new LAMA_FFS_Heuristic(lama_ff_heur);

		if(g_use_metric) {
			if(lama_ffc_heur == NULL)
				lama_ffc_heur = new LAMA_FFC_Heuristic(lama_ff_heur);

			// uses the correct heuristic depending on weight, but also
			// stores the ones not being used
			if(engine->get_weight() == GBFS) {
				engine->add_heuristic(lama_ffs_heur, true, use_ff_switch_prefs);
				engine->add_heuristic(lama_ffc_heur, false, false);
			} else {
				engine->add_heuristic(lama_ffs_heur, false, false);
				engine->add_heuristic(lama_ffc_heur, true, use_ff_switch_prefs);
			}
		} else {
			engine->add_heuristic(lama_ffs_heur, true, use_ff_switch_prefs);
		}
	}

	if(use_fd_switch) {
		if(lama_fd_ff_heur == NULL)
			lama_fd_ff_heur = new FDFFHeuristic(axiom_eval);

		if(g_use_metric && lama_ff_heur == NULL)
			lama_ff_heur = new FFHeuristic;
		if(g_use_metric && lama_ffc_heur == NULL)
			lama_ffc_heur = new LAMA_FFC_Heuristic(lama_ff_heur);

		if(g_use_metric) {
			// uses the correct heuristic depending on weight
			if(engine->get_weight() == GBFS) {
				engine->add_heuristic(lama_ffc_heur, false, false);
				engine->add_heuristic(lama_fd_ff_heur, true, use_fd_switch_prefs);
			} else {
				engine->add_heuristic(lama_ffc_heur, true, false);
				engine->add_heuristic(lama_fd_ff_heur, false, use_fd_switch_prefs);
			}
		} else {
			engine->add_heuristic(lama_fd_ff_heur, true, use_fd_switch_prefs);
		}
	}

    if(use_fd_ff || use_fd_ff_prefs) {

        if(lama_fd_ff_heur == NULL)
            lama_fd_ff_heur = new FDFFHeuristic(axiom_eval);
		
		engine->add_heuristic(lama_fd_ff_heur, use_fd_ff, use_fd_ff_prefs);
	}
	
	if(use_blind) {
	    engine->add_heuristic(new BlindSearchHeuristic, true, false);
	}

	if(use_goal_count) {
		engine->add_heuristic(new GoalCountHeuristic, true, false);
	}
}

