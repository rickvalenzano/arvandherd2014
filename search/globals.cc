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

#include "globals.h"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <unistd.h>

using namespace std;

#include "axioms.h"
#include "domain_transition_graph.h"
#include "operator.h"
#include "state.h"
#include "successor_generator.h"
#include "landmarks_graph.h"
#include "landmarks_graph_rpg_sasp.h"
#include "ff_heuristic.h"
#include "walker.h"
#include "mrw.h"
#include "aras_state.h"
#include "predecessor_generator.h"

bool check_goal(State* state){
	for (int i = 0; i < g_goal.size(); ++i) {
		if(state->operator [](g_goal[i].first) != g_goal[i].second)
			return false;
	}
	return true;
}

bool check_goal(ArasState* state) {
	for (int i = 0; i < g_goal.size(); ++i) {
		if (state->operator [](g_goal[i].first) != g_goal[i].second)
			return false;
	}
	return true;
}


// Make sure only being called by one thread at a time
bool validate_plan(const vector<const Operator *> &plan) {
    State s(*g_initial_state);
    for(int i = 0; i < plan.size(); i++) {
        if(!plan[i]->is_applicable(s)) {
            cerr << "Operator " << i << " is not applicable" << endl;
            exit(1);
        }
        State next_state(s, *plan[i], g_axiom_evaluator);
        s = next_state;
    }
    
    if(!check_goal(&s)) {
        cerr << "Path does not finish in a goal state" << endl;
	    exit(1);
    }
    
    return true;
}

pthread_mutex_t mutex_save_plan; // mutex for plan saving

// Make sure only being called by one thread at a time
int save_plan(const vector<const Operator *> &plan, const string& filename,
		const string &thread_name, bool dump_plan) {

	// only one thread can save a plan at any one time
	pthread_mutex_lock(&mutex_save_plan);

	ofstream outfile;
	
	if(validate_plan(plan))
	    cout << thread_name << "Plan Validated" << endl;
	
	int plan_cost = compute_cost(plan);
	if(g_best_sol_cost != -1 && plan_cost >= g_best_sol_cost) {
		cout << thread_name  << "Last complete plan has cost " << plan_cost <<
				" and is not being printed" << endl;

		pthread_mutex_unlock(&mutex_save_plan); // release lock
		return plan_cost;
	}
	g_best_sol_cost = plan_cost;

	bool separate_outfiles = g_iterative; // IPC conditions, change to false for a single outfile.
	if (separate_outfiles) {
		// Write a separat output file for each plan found by iterative search
		stringstream it_no;
		it_no << g_sol_number;
		outfile.open((filename + "." + it_no.str()).c_str(), ios::out);
        g_sol_number++;
	} else {
		// Write newest plan always to same output file
		outfile.open(filename.c_str(), ios::out);
	}
	
	State s(*g_initial_state);
	
	for (int i = 0; i < plan.size(); i++) {
		int action_cost = plan[i]->get_true_cost();

		if (dump_plan) {    
		    
			if (!g_use_metric)
				cout << thread_name << plan[i]->get_name() << endl;
			else
				cout << thread_name << plan[i]->get_name() << " (" <<
					action_cost << ")" << endl;
		    
	        State next_state(s, *plan[i], g_axiom_evaluator);
            s = next_state;
		}
		outfile << "(" << plan[i]->get_name() << ")" << endl;
	}
	outfile.close();
	if (dump_plan) {
		cout << thread_name << "Plan length: " << plan.size()
				<< " step(s), cost: " << plan_cost << "." << endl;
	}
	cout << thread_name << "Total time: " << g_timer << endl;
    cout << thread_name << "cost: " << plan_cost << "." << endl;

    // release lovk
    pthread_mutex_unlock(&mutex_save_plan);

	return plan_cost;
}


void check_magic(istream &in, string magic) {
    string word;
    in >> word;
    if(word != magic) {
        cout << "Failed to match magic word '" << magic << "'." << endl;
        cout << "Got '" << word << "'." << endl;
        exit(1);
    }
}

void read_metric(istream &in) {
  check_magic(in, "begin_metric");
  in >> g_use_metric;
  check_magic(in, "end_metric");
}

void read_variables(istream &in) {
    check_magic(in, "begin_variables");
    int count;
    in >> count;
    for(int i = 0; i < count; i++) {
        string name;
        in >> name;
        g_variable_name.push_back(name);
        int range;
        in >> range;
        g_variable_domain.push_back(range);
        int layer;
        in >> layer;
        g_axiom_layers.push_back(layer);
    }
    check_magic(in, "end_variables");
}

void read_goal(istream &in) {
    check_magic(in, "begin_goal");
    int count;
    in >> count;
    for(int i = 0; i < count; i++) {
        int var, val;
        in >> var >> val;
        g_goal.push_back(make_pair(var, val));
    }
    check_magic(in, "end_goal");
}

void dump_goal() {
    cout << "Goal Conditions:" << endl;
    for(int i = 0; i < g_goal.size(); i++)
        cout << "  " << g_variable_name[g_goal[i].first] << ": "
             << g_goal[i].second << endl;
}

void read_operators(istream &in) {
    int count;
    in >> count;
	g_min_action_cost = INT_MAX;
    for(int i = 0; i < count; i++)
        g_operators.push_back(Operator(in, false));
}

void read_axioms(istream &in) {
    int count;
    in >> count;
    for(int i = 0; i < count; i++)
        g_axioms.push_back(Operator(in, true));

    g_axiom_evaluator = new AxiomEvaluator;
    g_axiom_evaluator->evaluate(*g_initial_state);
    g_axiom_evaluator->evaluate(*g_initial_aras_state);

}

void build_landmarks_graph(bool reasonable_orders) {
    g_lgraph = new LandmarksGraphNew();
    g_lgraph->read_external_inconsistencies();
    if(reasonable_orders) {
	g_lgraph->use_reasonable_orders();
    }
    g_lgraph->generate();
    cout << "Generated " << g_lgraph->number_of_landmarks() << " landmarks, of which "
	 << g_lgraph->number_of_disj_landmarks() << " are disjunctive" << endl
	 << "          " << g_lgraph->number_of_edges() << " edges\n";
    //g_lgraph->dump();
}

void read_everything(istream &in, bool generate_landmarks, bool reasonable_orders) {
    read_metric(in);
    read_variables(in);
    g_initial_state = new State(in);
    g_initial_aras_state = new ArasState(*g_initial_state);
    //g_initial_state->dump();
    //g_initial_aras_state->dump();
    read_goal(in);
    read_operators(in);
    read_axioms(in);
    check_magic(in, "begin_SG");
    g_successor_generator = read_successor_generator(in);
    check_magic(in, "end_SG");
    DomainTransitionGraph::read_all(in);
    if(generate_landmarks){
	    build_landmarks_graph(reasonable_orders);
    }
    g_initial_state->set_landmarks_for_initial_state();
    g_predecessor_generator = new PredecessorGenerator;
}


void dump_everything() {
    cout << "Use metric? " << g_use_metric << endl;
    cout << "Variables (" << g_variable_name.size() << "):" << endl;
    for(int i = 0; i < g_variable_name.size(); i++)
        cout << "  " << g_variable_name[i]
             << " (range " << g_variable_domain[i] << ")" << endl;
    cout << "Initial State:" << endl;
    g_initial_state->dump();
    dump_goal();
    cout << "Successor Generator:" << endl;
    g_successor_generator->dump();
    for(int i = 0; i < g_variable_domain.size(); i++)
        g_transition_graphs[i]->dump();
}

int compute_cost(const vector<const Operator*>& plan){
	int cost = 0;
	for (int i = 0; i < plan.size(); ++i) {
		cost += plan[i]->get_true_cost();
	}
	return cost;
}

int get_peak_memory_in_kb() {
    // On error, produces a warning on cerr and returns -1.
    ostringstream filename_stream;
    filename_stream << "/proc/" << getpid() << "/status";
    const char *filename = filename_stream.str().c_str();

    ifstream procfile(filename);
    string word;
    while (procfile.good()) {
        procfile >> word;
        if (word == "VmPeak:") {
            int memory_kb;
            procfile >> memory_kb;
            if (procfile.fail())
                break;
            return memory_kb;
        }
        // Skip to end of line.
        procfile.ignore(numeric_limits<streamsize>::max(), '\n');
    }
    cerr << "warning: error reading memory from procfile" << endl;
    return -1;
}

void print_peak_memory() {
    cout << "Peak memory: " << get_peak_memory_in_kb() << " KB" << endl;
}

int get_current_seed(int seed_increment) {

	int next_seed = g_seed_list[g_seed_index];

	if(g_seed_index == g_seed_list.size() - 1)
		g_seed_list[g_seed_list.size() - 1] += seed_increment;
	else
		g_seed_index++;
	return next_seed;
}

bool g_use_metric;
vector<string> g_variable_name;
vector<int> g_variable_domain;
vector<int> g_axiom_layers;
vector<int> g_default_axiom_values;
State *g_initial_state;
vector<pair<int, int> > g_goal;
vector<Operator> g_operators;
vector<Operator> g_axioms;
AxiomEvaluator *g_axiom_evaluator;
SuccessorGenerator *g_successor_generator;
vector<DomainTransitionGraph *> g_transition_graphs;

LandmarksGraph *g_lgraph;

string g_all_groups_file;
string g_output_filename;
int g_sol_number = 1;

Timer g_timer;

vector<MRW_Parameters*> g_params_list;

ArasState *g_initial_aras_state;
int g_min_action_cost;
PredecessorGenerator *g_predecessor_generator;
Shared_MRW_Parameters *g_mrw_shared = NULL;

UCB* p_learner = NULL;
int g_best_sol_cost = -1;
ClosedList<State, const Operator *> g_closed_list;

WalkPool* g_walk_pool = NULL;
vector<const Operator *> g_init_trajectory;

const char * g_init_trajectory_file = NULL;

pthread_mutex_t mutex_p_learner;
pthread_mutex_t mutex_w_pool;

vector<int> g_seed_list;
int g_seed_index = 0;

bool g_iterative = false;
