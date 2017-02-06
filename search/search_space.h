#ifndef SEARCH_SPACE_H
#define SEARCH_SPACE_H

#include <vector>
#define LOADING_FACTOR 2
#define PER_NODE_OVERHEAD 12
using namespace std;
class Operator;
class ArasState;
class SearchNodeInfo;


class SearchNode {
    unsigned short *state_buffer;
    SearchNodeInfo &info;
    size_t* parents_num;

public:
    SearchNode(unsigned short *state_buffer_, SearchNodeInfo &info_, size_t* parents_num_byte);

    unsigned short *get_state_buffer() {
      return state_buffer;
    }
    ArasState get_state() const;

    bool is_goal() const;
    bool is_open() const;
    bool is_closed() const;
    bool is_dead_end() const;
    bool is_expanded() const;
    bool is_reg_expanded() const;
    bool is_new() const;

    int get_f() const;
    int get_g() const;
    int get_h() const;
    int get_level() const;
    int get_parent_num() const;
    
    void set_level(int  l);

	void add_parent(const SearchNode &parent_node, const Operator *parent_op);
	void make_permanent();
	void make_reg_permanent();
	
	
	void lazy_open(int h, const SearchNode &parent_node, const Operator *parent_op);
	void lazy_reopen(const SearchNode &parent_node, const Operator *parent_op);

	void update_and_open(int h, int g, int op_cost);
	void update_and_reopen(int g, int op_cost);
    
    void open_initial(int h);
    void open(int h, const SearchNode &parent_node, const Operator *parent_op);
    void reopen(const SearchNode &parent_node, const Operator *parent_op);
    
    void open(int h, const SearchNode &parent_node, const Operator *parent_op, int op_cost);
    void reopen(const SearchNode &parent_node, const Operator *parent_op, int op_cost);

    void close();
    void mark_as_dead_end();
    /*const vector<unsigned short *>& get_parent_states();
    const vector<const Operator *>& get_creating_operator();*/
    const vector<pair<unsigned short *, const Operator *> > get_parents();


    void dump();
};

//class BoostingNode : public SearchNode{
	
// };

class SearchSpace {
    class HashTable;
    HashTable *nodes;
    bool keep_shallow_copy;
    size_t parents_num;
public:
    SearchSpace();
    ~SearchSpace();
    int size() const;
    size_t memory_usage() const;
    
    SearchNode get_node(const ArasState &state);
    // SearchNode get_itsa_node(const ArasState &state);
    void trace_path(const ArasState &goal_state,
		    std::vector<const Operator *> &path) const;

    void set_shallow(){
    	keep_shallow_copy = true;
    }
    void dump();
    void statistics() const;
};



#endif
