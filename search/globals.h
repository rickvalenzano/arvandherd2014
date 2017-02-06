/*********************************************************************
 * Author: Malte Helmert (helmert@informatik.uni-freiburg.de)
 * (C) Copyright 2003-2004 Malte Helmert
 * Modified by: Silvia Richter (silvia.richter@nicta.com.au),
 *              Matthias Westphal (westpham@informatik.uni-freiburg.de)             
 * (C) Copyright 2008 NICTA and Matthias Westphal
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

#ifndef GLOBALS_H
#define GLOBALS_H

#include <iostream>
#include <string>
#include <vector>
#include "timer.h"
#include "mrw_parameters.h"
#include "shared_mrw_parameters.h"
#include "parameter_learner.h"
#include "closed_list.h"

#include <limits.h>

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <limits>
#include <sstream>
#include <pthread.h>


using namespace std;

class AxiomEvaluator;
class Cache;

class DomainTransitionGraph;
class Operator;
class Axiom;
class State;
class SuccessorGenerator;
class FFHeuristic;
class LandmarksCountHeuristic;
class LandmarksGraph;
// class PPT;
class DDTNode;
class MRW_Parameters;
class ArasState;
class PredecessorGenerator;
class WalkPool;

// reads everything from file
void read_everything(istream &in, bool generate_landmarks,
		bool reasonable_orders);
void dump_everything();

// checks if the given state is a goal
bool check_goal(State* state);

// checks if the given ArasState is a goal
bool check_goal(ArasState* state);

// computes the cost of a plan
int compute_cost(const vector<const Operator*>& plan);

// used for string manipulation
void check_magic(istream &in, string magic);

// saves the plan to the given filename
int save_plan(const vector<const Operator *> &plan, const string& filename,
		const string &thread_name, bool dump_plan = false);

// returns the peak amount of memory used so far in kb
int get_peak_memory_in_kb();

// prints the peak amount of memory to stdout
void print_peak_memory();

// get current seed. Only increments if we have run out of seeds
int get_current_seed(int seed_increment);

struct hash_operator_ptr {
    size_t operator()(const Operator *key) const {
	return reinterpret_cast<unsigned long>(key);
    }
};

// are there non-unit costs - not changed after initialization in global
extern bool g_use_metric;

// variable names - not changed after initialization in global
extern vector<string> g_variable_name;

// not sure what this is - not changed after initialization in global
extern vector<int> g_variable_domain;

// not sure what this is - not changed after initialization in global
extern vector<int> g_axiom_layers;

// not sure what this is - aras_state uses it for something (should lock down)
extern vector<int> g_default_axiom_values;

// initial state - need to be very careful with it, perhaps copy more often
extern State *g_initial_state;

// goal variable values - not changed after initialization in global
extern vector<pair<int, int> > g_goal;

// holds the operators - probably isn't changed after initialization in global
extern vector<Operator> g_operators;

// not sure what this is - not changed after initialization in global
extern vector<Operator> g_axioms;

// Need to give each thread a separate evaluator
extern AxiomEvaluator *g_axiom_evaluator;

// only method-const methods called after construction
extern SuccessorGenerator *g_successor_generator;

// used for CG heuristic and building landmark graph - since only built at 
// beginning, should be fine
extern vector<DomainTransitionGraph *> g_transition_graphs;

// shared landmarks graph - all public functions are const-methods, so no worries
// used primarily by LM heuristic, which seems to copy it anyways, so probably
// not a big deal
extern LandmarksGraph *g_lgraph;

// only used once to build everything up
extern string g_all_groups_file;

// main file name to output to, g_sol_number is added to the back
extern string g_output_filename;
extern int g_sol_number;

// only used by planner and plan_booster
extern Timer g_timer;

// list of mrw parameters
extern vector<MRW_Parameters*> g_params_list;

// TODO - Check this out
extern ArasState *g_initial_aras_state;
extern int g_min_action_cost;
extern PredecessorGenerator *g_predecessor_generator;

// shared mrw parameters
extern Shared_MRW_Parameters *g_mrw_shared;

// parameter learner for mrw
extern UCB* p_learner;
extern pthread_mutex_t mutex_p_learner;

// This is the actual cost (not the adjusted cost) of the best solution found
// so far. Initialized as 0
extern int g_best_sol_cost;

// best first search closed list.
extern ClosedList<State, const Operator *> g_closed_list;

// pool for mrw walks
extern WalkPool* g_walk_pool;
extern pthread_mutex_t mutex_w_pool;

// name of init trajectory file and actual trajectory
extern const char * g_init_trajectory_file;
extern vector<const Operator *> g_init_trajectory;

extern vector<int> g_seed_list;
extern int g_seed_index;

extern bool g_iterative;

#endif
