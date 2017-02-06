/*********************************************************************
 * Author: Malte Helmert (helmert@informatik.uni-freiburg.de)
 * (C) Copyright 2003-2004 Malte Helmert
 * Modified by: Silvia Richter (silvia.richter@nicta.com.au)
 * (C) Copyright 2008 NICTA
 *
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

#include "delayed_wa_star.h"

#include "globals.h"
#include "heuristic.h"
#include "successor_generator.h"
#include "operator.h"
#include "ff_heuristic.h"
#include "landmarks_count_heuristic.h"
#include "memory_utils.h"

#include <cassert>
#define RAND_H_RANGE 0

using namespace std;

OpenListDelayedInfo::OpenListDelayedInfo(Heuristic *heur, bool only_pref) {
	heuristic = heur;
	only_preferred_operators = only_pref;
	priority = 0;
}

OpenListDelayedEntry::OpenListDelayedEntry(const State *_parent,
		const Operator *_op) {
	parent = _parent;
	op = _op;
}

DelayedWAStar::DelayedWAStar(AxiomEvaluator *a, int s_num, MTRand_int32 *rg,
		int w, string n, bool ignore, bool r, double eps) :
	axiom_eval(a), current_state(*g_initial_state), rand_gen(rg),
	name(n), epsilon(eps) {

	generated_states = 0;
	current_predecessor = 0;
	current_operator = 0;

    kb_limit = -1.0;
    expansion_limit = -1;
    ignore_costs = ignore;
    rand_open = r;
    search_num = s_num;
    pref_priority_reward = 1000;
    
    // clearing vectors
    heuristics.clear();
    preferred_operator_heuristics.clear();
    open_lists.clear();
    best_heuristic_values.clear();
    
    g_preferred_operators.clear();
    
    weight = w;

    use_local_bound = false;
    local_bound = -1;
}

DelayedWAStar::~DelayedWAStar() {}

void DelayedWAStar::add_heuristic(Heuristic *heuristic, bool use_estimates, bool use_preferred_operators) {
	heuristics.push_back(heuristic);
	cout << "Adding Heuristic: " << heuristic->get_heuristic_name();
	
	best_heuristic_values.push_back(-1);
	if (use_estimates) {
		cout << ", using estimates";
		open_lists.push_back(OpenListDelayedInfo(heuristic, false));
		open_lists.push_back(OpenListDelayedInfo(heuristic, true));
	}
	if (use_preferred_operators) {
		preferred_operator_heuristics.push_back(heuristic);
		cout << ", using preferred ops";
	}
	cout << endl;
}

void DelayedWAStar::initialize() {
    
    if(weight == GBFS)
        cout << name << "Conducting delayed greedy best-first search" << endl;
    else
        cout << name << "Conducting delayed WA* search with weight " << weight
        	<< endl;
    
    if(RAND_H_RANGE > 0)
	    cout << name << "\tUsing rand_h range of " << RAND_H_RANGE << endl;
	    
	assert(!open_lists.empty());
	
	generated_states = 0;
	expanded_states = 0;
	dead_end_count = 0;
	
    current_predecessor = 0;
    current_operator = 0;
}

void DelayedWAStar::statistics() const {
    cout << name << "Expanded " << expanded_states <<
    		" state(s) in last iteration." << endl;
    cout << name << "Generated " << generated_states <<
    		" state(s) in last iteration" << endl;
    cout << name << "Total num of dead-ends in last iteration is " <<
    		dead_end_count << " state(s)." << endl;
	cout << name << "Total closed list size is " << g_closed_list.size() <<
			" state(s)." << endl;
	
}

bool DelayedWAStar::expand_closed_node(const State *parent_ptr) {

    // if is a BFS
    if(weight == GBFS)
        return parent_ptr->get_search_num() < search_num;
    else if(parent_ptr->get_search_num() < search_num) {// if is a WA*
    	if(ignore_costs)
    		return (current_state.get_depth() <= parent_ptr->get_depth());
    	else
    		return (current_state.get_g_value() <= parent_ptr->get_g_value());
    }

    return false;
}

// Invariants:
// - current_state is the next state for which we want to compute the heuristic.
// - current_predecessor is a permanent pointer to the predecessor of that state.
// - current_operator is the operator which leads to current_state from predecessor.
int DelayedWAStar::step() {

	// Return if some other thread found a solution and shouldn't
	// keep going
	if(!g_iterative && g_best_sol_cost != -1)
		return SOLVED;

	// Evaluate only if g-cost of state is lower than bound
	if(use_local_bound && local_bound != -1) {
		if(ignore_costs && current_state.get_depth() >= local_bound)
			return fetch_next_state();
		else if(!ignore_costs && current_state.get_g_value() >= local_bound)
			return fetch_next_state();
	} else if(!use_local_bound && g_best_sol_cost != -1) {
		if(ignore_costs && current_state.get_depth() >= g_best_sol_cost)
			return fetch_next_state();
		else if(!ignore_costs && current_state.get_g_value() >= g_best_sol_cost)
			return fetch_next_state();
	}
    
    bool expand_node = false;
    bool found_better_path = false;
    bool not_this_iter = false;

    const State *parent_ptr = 0;
    // if not in closed list
	if (!g_closed_list.contains(current_state)) {
	
	    calculate_heuristics_and_store();
        
		parent_ptr = g_closed_list.insert(current_state, 
                current_predecessor, current_operator);

        g_closed_list.update_num_bytes(current_state.approx_num_bytes());
        expand_node = true;
		
	} else {
	
	    parent_ptr = g_closed_list.find(current_state);
	    
	    if((ignore_costs && current_state.get_depth() < parent_ptr->get_depth()) ||
	    		(!ignore_costs && current_state.get_g_value() < parent_ptr->get_g_value()))
	    	found_better_path = true;

	    if(parent_ptr->get_search_num() < search_num)
	    	not_this_iter = true;

	    // if should expand an already closed node (depends on the algorithm)
	    if(found_better_path || not_this_iter) {
			// We need a const_cast here, as we have to modify parent, but the 
			// STL Map underlying closed_list returns a const_iterator. However, cast
			// is safe as modification of parent does not effect its position in closed_list.
			State *modifiable_parent_ptr = const_cast<State*> (parent_ptr);
			
			// Change g-value and reached landmarks in state
			// NOTE: Landmarks change, but currently, we aren't taking this into
			// account for the heuristic. Instead, we just use cached value
			
			// only changing ancestor if leads to shorter path
			if (found_better_path) {
			    modifiable_parent_ptr->change_ancestor(*current_predecessor, *current_operator);
			    g_closed_list.update(current_state, current_predecessor, current_operator);

			    if(weight != GBFS)
			    	expand_node = true;
	        }
	    
			// update search number
			if(not_this_iter) {
				modifiable_parent_ptr->set_search_num(search_num);
				expand_node = true;
			}
    	    
	    }

	    if(expand_node) {
	    	// TODO Should also check which heuristics need to be computed here
	    	// in case have changed set of heuristics
	    	get_preferred_from_closed_list(parent_ptr);
	    }

	}
	
	if (expand_node) {
	
	    expanded_states++;
	    
	    if(expansion_limit > 0 && expanded_states > expansion_limit) {
	    	return OUT_OF_TIME;
	    }

	    if(expanded_states % 100 == 0 && kb_limit != -1.0) {
	    	//double vm, rss;
	    	//process_mem_usage(vm, rss);

	    	//if(vm > 0.0 && vm > byte_limit)
	    	//	return OUT_OF_MEMORY;
	    	if(memory_estimate()/1000 > kb_limit) {
	    		return OUT_OF_MEMORY;
	    	}
	    }

	    if(parent_ptr->is_dead_end()) {
	        dead_end_count++;
	    } else {
		    if (check_goal(parent_ptr)) {
		            
			    return SOLVED;
		    } else if (check_progress(parent_ptr)) {
			    report_progress();
			    reward_progress();
		    }
		    
		    generate_successors(parent_ptr);
		}
	}
	
	return fetch_next_state();
}

bool DelayedWAStar::check_goal(const State *state_ptr) {
	// Any heuristic reports 0 if this is a goal state, so we can
	// pick an arbitrary one.
	Heuristic *heur = open_lists[0].heuristic;
	if (!state_ptr->is_dead_end(heur->get_heuristic_name()) && 
	        state_ptr->get_heuristic_value(heur->get_heuristic_name()) == 0) {
		// We actually need this silly !heur->is_dead_end() check because
		// this state *might* be considered a non-dead end by the
		// overall search even though heur considers it a dead end
		// (e.g. if heur is the CG heuristic, but the FF heuristic is
		// also computed and doesn't consider this state a dead end.
		// If heur considers the state a dead end, it cannot be a goal
		// state (heur will not be *that* stupid). We may not call
		// get_heuristic() in such cases because it will barf.

		// If (and only if) using action costs the heuristic might report 0
		// even though the goal is not reached - check.
		if (g_use_metric)
			for (int i = 0; i < g_goal.size(); i++)
				if (current_state[g_goal[i].first] != g_goal[i].second)
					return false;
		// cout << "Solution found!" << endl;
		Plan plan;
		g_closed_list.trace_path(current_state, plan);
		set_plan(plan);
		return true;
	} else {
		return false;
	}
}

bool DelayedWAStar::check_progress(const State *state_ptr) {
	bool progress = false;
	for (int i = 0; i < heuristics.size(); i++) {
		if (state_ptr->is_dead_end(heuristics[i]->get_heuristic_name()))
			continue;
		int h = state_ptr->get_heuristic_value(heuristics[i]->get_heuristic_name());
		assert(h>=0);
		int &best_h = best_heuristic_values[i];
		if (best_h == -1 || h < best_h) {
			best_h = h;
			progress = true;
		}
	}
	return progress;
}

void DelayedWAStar::report_progress() {
	cout << name  << "Best h values: ";
	for (int i = 0; i < heuristics.size(); i++) {
		cout << best_heuristic_values[i];
		if (i != heuristics.size() - 1)
			cout << "/";
	}
	cout << "[exp: " << expanded_states 
	        << ", gen: " << generated_states
	        << ", dead ends: " << dead_end_count
	        << ", closed size: " << g_closed_list.size() << "]" << endl;
}

void DelayedWAStar::reward_progress() {
	// Boost the "preferred operator" open lists somewhat whenever
	// progress is made. This used to be used in multi-heuristic mode
	// only, but it is also useful in single-heuristic mode, at least
	// in Schedule.
	//
	// TODO: Test the impact of this, and find a better way of rewarding
	// successful exploration. For example, reward only the open queue
	// from which the good state was extracted and/or the open queues
	// for the heuristic for which a new best value was found.

    if(pref_priority_reward != 0) {
	    for (int i = 0; i < open_lists.size(); i++)
		    if (open_lists[i].only_preferred_operators)
			    open_lists[i].priority -= pref_priority_reward;
    }
}

void DelayedWAStar::calculate_heuristics_and_store() {
    
    for (int i = 0; i < heuristics.size(); i++) {
        heuristics[i]->set_recompute_heuristic(current_state);
    }
    
    for (int i = 0; i < heuristics.size(); i++) {
		heuristics[i]->evaluate(current_state);
	    
        if (!heuristics[i]->is_dead_end()) {
            current_state.add_heuristic_value(heuristics[i]->get_heuristic_name(), 
                heuristics[i]->get_heuristic());

        } else {

            current_state.add_heuristic_value(heuristics[i]->get_heuristic_name(), 
                Heuristic::DEAD_END);
            
            if (heuristics[i]->dead_ends_are_reliable())
                current_state.record_as_dead_end();
        
        }       
    }
       
    // gets the preferred operators   
    g_preferred_operators.clear();
    vector<const Operator *> prefs;
    
	for (int i = 0; i < preferred_operator_heuristics.size(); i++) {
	    prefs.clear();
		Heuristic *heur = preferred_operator_heuristics[i];
		if (!heur->is_dead_end()) {
			heur->get_preferred_operators(g_preferred_operators);
			heur->get_preferred_operators(prefs);
			current_state.add_preferred_ops(heur->get_heuristic_name(), prefs);
	    }
	    
	}
    
    current_state.set_search_num(search_num);
}

void DelayedWAStar::get_preferred_from_closed_list(const State *state_ptr) {

    g_preferred_operators.clear();
    
    for (int i = 0; i < preferred_operator_heuristics.size(); i++) {
        Heuristic *heur = preferred_operator_heuristics[i];
		if (!state_ptr->is_dead_end(heur->get_heuristic_name()))
		    state_ptr->get_preferred_ops(heur->get_heuristic_name(), g_preferred_operators);
    }

}

void DelayedWAStar::generate_successors(const State *parent_ptr) {
	vector<const Operator *> all_operators;
	g_successor_generator->generate_applicable_ops(current_state, all_operators);
    
    int parent_g = parent_ptr->get_g_value();
    int depth = parent_ptr->get_depth() + 1;
    
	for (int i = 0; i < open_lists.size(); i++) {
		Heuristic *heur = open_lists[i].heuristic;
		
		if (!parent_ptr->is_dead_end(heur->get_heuristic_name())) {
		    int h = parent_ptr->get_heuristic_value(heur->get_heuristic_name());
			
			OpenList<OpenListDelayedEntry> &open = open_lists[i].open;
			vector<const Operator *> &ops = open_lists[i].only_preferred_operators ? g_preferred_operators : all_operators;
            
            if(rand_open && ops.size() > 1) {
                for(unsigned j = 0; j < ops.size() - 1; j++) {
                    unsigned index = rand_gen->get_32bit_int() % (ops.size() - j);
                    const Operator *to_move = ops[j];
                    ops[j] = ops[index + j];
                    ops[index + j] = to_move;
                }
            }
			
            for (int j = 0; j < ops.size(); j++) {
				int g_cost = parent_g + ops[j]->get_cost();
			    int my_h = h;
			    
			    if(RAND_H_RANGE)
			        my_h += (rand_gen->get_32bit_int() % (2*RAND_H_RANGE + 1)) 
			                - RAND_H_RANGE;
			    
			    int my_cost;
			    if(weight == GBFS)
			        my_cost = my_h;
			    else {
			    	if(ignore_costs)
			    		my_cost = weight * my_h + depth;
			    	else
			    		my_cost = weight * my_h + g_cost;
			    }

			    int tie_breaker;
			    if(weight == GBFS) {
			    	if(ignore_costs)
			    		tie_breaker = depth;
			    	else
			    		tie_breaker = g_cost;
			    } else
			    	tie_breaker = my_h;

				open.insert(make_pair(my_cost, tie_breaker), OpenListDelayedEntry(parent_ptr, ops[j]));
			}
		}
	}
	generated_states += all_operators.size();
}

int DelayedWAStar::fetch_next_state() {
	OpenListDelayedInfo *open_info = select_open_queue();
	if (!open_info) {
		cout << name << "Completely explored state space -- no solution!" << endl;
		return FAILED;
	}

	OpenListDelayedEntry next;
	if(epsilon == 0.0 || rand_gen->get_double() > epsilon) {
		next = open_info->open.remove_min();
	} else {
		next = open_info->open.remove_rand_node(rand_gen);
	}
	open_info->priority++;

	current_predecessor = next.parent;
	current_operator = next.op;
	current_state = State(*current_predecessor, *current_operator, axiom_eval);

	return IN_PROGRESS;
}

OpenListDelayedInfo *DelayedWAStar::select_open_queue() {
	OpenListDelayedInfo *best = 0;
	for (int i = 0; i < open_lists.size(); i++)
		if (!open_lists[i].open.empty() && (best == 0 || open_lists[i].priority < best->priority)) {
			best = &open_lists[i];
		}
	return best;
}

size_t DelayedWAStar::memory_estimate() const {

    size_t size = 0;
    for(int i = 0; i < open_lists.size(); i++) {
        size += open_lists[i].open.approx_num_bytes();
    }
    size += g_closed_list.approx_num_bytes();
    
    return size;
}

