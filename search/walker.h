#ifndef WALKER_H_
#define WALKER_H_
#include <vector>
#include <set>
//#include "set"
#include "globals.h"
#include "state.h"
#include "operator.h"
#include "heuristic.h"
#include "mrw.h"
#include "fd_ff_heuristic.h"
#include "mtrand.h"

using namespace std;


// This class is designed to run random_walks. Based on the given parameters
// action selection can be biased towards some actions and away some other. 
class MRW_Parameters;
class SearchNode;
class DDTNode;

struct WalkInfo{
	int branching;
	int length_offset;
	vector<const Operator*> path;
	bool goal_visited;
	int value;
	int cost;
};

class Walker
{
	vector<int> num;
	
	WalkInfo my_info;
	Heuristic* heuristic;
    int walk_type;

    AxiomEvaluator *axiom_eval;

    MTRand_int32 *rand_gen;

    const Operator* random_successor(vector<const Operator *>& applicable_ops);
	const Operator* mha_successor(vector<const Operator *>& applicable_ops, MRW_Parameters& param);
	const Operator* mda_successor(vector<const Operator *>& applicable_ops, MRW_Parameters& param, bool debug);
	
	double gibbs_func(double avg, float temperature);
	void update_mda_action_values();
	void update_mha_action_values(vector<const Operator *> &helpful_actions);
public:
	vector<double> Q_MHA;
	vector<double> Q_MDA;
	int num_evaluated;
	
	void dump_mha_scores();
    Walker(AxiomEvaluator *a, MTRand_int32 *r): axiom_eval(a), rand_gen(r){}
    void init_info();

    WalkInfo get_info();
	
    void random_walk(State initial_state, int length, MRW_Parameters& params, int cost_bound, bool debug);
	void op_info();

    /**
     * Prepares the walker for a series of walks during which information will
     * be reused.
    **/ 
    void prepare_for_walks(MRW_Parameters& params, Heuristic *h);
	virtual ~Walker();
};

#endif /*WALKER_H_*/
