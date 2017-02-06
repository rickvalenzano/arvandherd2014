/*********************************************************************
 * Author: Malte Helmert (helmert@informatik.uni-freiburg.de)
 * (C) Copyright 2003-2004 Malte Helmert
 * Modified by: Matthias Westphal (westpham@informatik.uni-freiburg.de)
 * (C) Copyright 2008 Matthias Westphal
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

#ifndef HEURISTIC_H
#define HEURISTIC_H

#include <map>
#include <vector>
#include <string>

class Operator;
class State;


class Heuristic {

    enum {INVALID = -2};

protected:
    std::vector<const Operator *> preferred_operators;

	int heuristic;
    virtual int compute_heuristic(const State &state) = 0;
    void set_preferred(const Operator *op);
public:
	enum {DEAD_END = -1};
    Heuristic();
    virtual ~Heuristic();

    void evaluate(const State &state);
    bool is_dead_end();
    int get_heuristic();
    void get_preferred_operators(std::vector<const Operator *> &result);
    virtual void set_recompute_heuristic(const State &state);
    virtual bool dead_ends_are_reliable() {return true;}
    virtual std::string get_heuristic_name() = 0;
    
    void clear_preferred_ops(){preferred_operators.clear();}
};

#endif
