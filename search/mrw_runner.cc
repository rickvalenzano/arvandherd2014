#include "globals.h"
#include "axioms.h"

#include "mrw_runner.h"
#include "landmarks_graph.h"
#include "string.h"
#include "mrw.h"

#include "ff_heuristic.h"
#include "fd_ff_heuristic.h"
#include "lama_ffc_heuristic.h"
#include "lama_ffs_heuristic.h"
#include "blind_search_heuristic.h"
#include "landmarks_count_heuristic.h"
#include "goal_count_heuristic.h"

void fix_mrw_configs();
void add_heuristics(MRW* engine, AxiomEvaluator *axiom_eval);

void run_mrw_search(bool finish_mrw_before_exit) {

    // initialize parameter learner
    p_learner = new UCB(g_mrw_shared->ucb_const, g_mrw_shared->adjust_online,
    		new MTRand_int32(get_current_seed(11)));

    // create pool for smart restarts
	if(g_mrw_shared->restart_type == Shared_MRW_Parameters::S_RESTART){
	    g_walk_pool = new WalkPool(g_mrw_shared->pool_size, 
                g_mrw_shared->act_level, new MTRand_int32(get_current_seed(29)));
    }

    fix_mrw_configs();

    int num_to_run = g_mrw_shared->num_threads;
    if(finish_mrw_before_exit)
    	num_to_run--;

    pthread_t *mrw_threads = new pthread_t[num_to_run];
    struct mrw_thread_data *thread_data = new mrw_thread_data[num_to_run];

    for(int i = 0; i < num_to_run; i++) {
    	thread_data[i].seed = get_current_seed(1984);
    	thread_data[i].name = "$MRW" + int_to_string(i) + " - ";

    	int rc = pthread_create(&mrw_threads[i], NULL, run_mrw_thread,
    			(void *)&thread_data[i]);
    	if(rc) {
    		cerr << "ERROR: Return code from pthreade_create is " << rc << endl;
    	    exit(-1);
    	}
    }

    if(finish_mrw_before_exit) {
    	struct mrw_thread_data my_data;
    	my_data.seed = get_current_seed(1984);

    	if(num_to_run > 0)
    		my_data.name = "$MRW" + int_to_string(num_to_run) + " - ";
    	else
    		my_data.name = "";

    	//TODO Clean this up after competition deadline since just duplicating below
    	AxiomEvaluator *axiom_eval = new AxiomEvaluator;
    	MRW * engine = new MRW(axiom_eval, new MTRand_int32(my_data.seed),
    			my_data.name);
    	add_heuristics(engine, axiom_eval);

    	engine->search();
    }
}

void *run_mrw_thread(void *data){
	struct mrw_thread_data *my_data = (struct mrw_thread_data *)data;

	AxiomEvaluator *axiom_eval = new AxiomEvaluator;
	MRW * engine = new MRW(axiom_eval, new MTRand_int32(my_data->seed),
			my_data->name);
	add_heuristics(engine, axiom_eval);

	engine->search();
	pthread_exit(NULL);
}

void fix_mrw_configs() {
    // Check whether landmarks were found, fix parameter settings
    if (g_lgraph != NULL && g_lgraph->number_of_landmarks() == 0) {
        cout << "All landmark configs changed to use FD_FF" << endl;
		
		// Any mrw parameters using LM are changed to use FD_FF
        for(int i = 0; i < g_params_list.size(); i++) {
            if(g_params_list[i]->heur == MRW_Parameters::LM)
                g_params_list[i]->heur = MRW_Parameters::FD_FF;
        }
        
    }  
}

void add_heuristics(MRW* engine, AxiomEvaluator *axiom_eval){

        // heuristics to use in mrw
    bool mrw_fd_ff_heuristic = false;
    bool mrw_landmarks_heuristic = false;
    bool mrw_lama_ff_heuristic = false;
    bool mrw_lama_ff_s_heuristic = false;
    bool mrw_lama_ff_c_heuristic = false;
    bool mrw_blind_heuristic = false;
    bool mrw_goal_count_heuristic = false;
    
    bool need_lm_preferred = false;

    // make sure generate all applicable heuristics
    for(int i = 0; i < g_params_list.size(); i++) {

        if(g_params_list[i]->heur == MRW_Parameters::FD_FF) {
            mrw_fd_ff_heuristic = true;
        } else if(g_params_list[i]->heur == MRW_Parameters::LM) {
            mrw_landmarks_heuristic = true;

            if(g_params_list[i]->walk_type == MRW_Parameters::MHA)
            	need_lm_preferred = true;

        } else if(g_params_list[i]->heur == MRW_Parameters::LAMA_FF) {
            mrw_lama_ff_heuristic = true;
        } else if(g_params_list[i]->heur == MRW_Parameters::LAMA_FF_S) {
            mrw_lama_ff_s_heuristic = true;
        } else if(g_params_list[i]->heur == MRW_Parameters::LAMA_FF_C) {
            mrw_lama_ff_c_heuristic = true;
        } else if(g_params_list[i]->heur == MRW_Parameters::BLIND) {
            mrw_blind_heuristic = true;
        } else if(g_params_list[i]->heur == MRW_Parameters::GOAL_COUNT) {
        	mrw_goal_count_heuristic = true;
        } else {

            cerr << "Should never be another kind of heuristic" << endl;
            exit(1);
        }
    }

    // heuristic indices
    int ff_index = -1;
    int landmarks_index = -1;
    int lama_ff_index = -1; 
    int lama_ff_s_index = -1;
    int lama_ff_c_index = -1;
    int blind_index = -1;
    int goal_count_index = -1;

    int h_index = 0;

    if(mrw_fd_ff_heuristic) {    
        ff_index = h_index;
        h_index++;
    }  
    if(mrw_landmarks_heuristic) {
        landmarks_index = h_index;
        h_index++;
    } 
    if(mrw_lama_ff_heuristic) {
        lama_ff_index = h_index;
        h_index++;
    }
    if(mrw_lama_ff_s_heuristic) {
        lama_ff_s_index = h_index;
        h_index++;
    }
    if(mrw_lama_ff_c_heuristic) {
        lama_ff_c_index = h_index;
        h_index++;
    }
    if(mrw_blind_heuristic) {
        blind_index = h_index;
        h_index++;
    }
    if(mrw_goal_count_heuristic) {
    	goal_count_index = h_index;
    	h_index++;
    }

    // record index of heuristic in mrw
    for(int i = 0; i < g_params_list.size(); i++) {
        if(g_params_list[i]->heur == MRW_Parameters::FD_FF)
            g_params_list[i]->heur_index = ff_index;
        else if(g_params_list[i]->heur == MRW_Parameters::LM)
            g_params_list[i]->heur_index = landmarks_index;
        else if (g_params_list[i]->heur == MRW_Parameters::LAMA_FF)
            g_params_list[i]->heur_index = lama_ff_index;
        else if (g_params_list[i]->heur == MRW_Parameters::LAMA_FF_S)
            g_params_list[i]->heur_index = lama_ff_s_index;
        else if (g_params_list[i]->heur == MRW_Parameters::LAMA_FF_C)
            g_params_list[i]->heur_index = lama_ff_c_index;
        else if (g_params_list[i]->heur == MRW_Parameters::BLIND)
            g_params_list[i]->heur_index = blind_index;
        else if (g_params_list[i]->heur == MRW_Parameters::GOAL_COUNT)
            g_params_list[i]->heur_index = goal_count_index;
    }
    
    if(mrw_fd_ff_heuristic) {    
        engine->add_heuristic(new FDFFHeuristic(axiom_eval));
    }  
    if(mrw_landmarks_heuristic) {
        LandmarksCountHeuristic *mrw_lm_heur = new LandmarksCountHeuristic(*g_lgraph,
        		need_lm_preferred, new FFHeuristic);
		engine->add_heuristic(mrw_lm_heur);
    } 
    if(mrw_lama_ff_heuristic) {
        engine->add_heuristic(new FFHeuristic);
    }
    if(mrw_lama_ff_s_heuristic) {
        engine->add_heuristic(new LAMA_FFS_Heuristic(new FFHeuristic));
    }
    if(mrw_lama_ff_c_heuristic) {
        engine->add_heuristic(new LAMA_FFC_Heuristic(new FFHeuristic));
    }
    if(mrw_blind_heuristic) {
        engine->add_heuristic(new BlindSearchHeuristic);
    }
    if(mrw_goal_count_heuristic) {
    	engine->add_heuristic(new GoalCountHeuristic);
    }
}

