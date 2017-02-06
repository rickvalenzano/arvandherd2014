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

#ifndef OPEN_LIST_H
#define OPEN_LIST_H

#include <vector>
#include <queue>
#include "mtrand.h"

template<class Entry>
class OpenList {
    class IsWorse {
	public:
	bool operator() (const pair<pair<int,int>, Entry>& a, const pair<pair<int,int>, Entry>& b) const {
	    if(a.first.first != b.first.first)
		return a.first.first > b.first.first;
	    else
		return a.first.second > b.first.second;
	};
    };

    std::vector<pair<pair<int,int>, Entry> > the_heap;

    bool HeapifyUp(unsigned int index);
    bool HeapifyDown(unsigned int index);

public:
    OpenList();
    ~OpenList();

    size_t memory_usage() {return sizeof(pair<pair<int,int>, Entry>) * the_heap.size() * 2;}

    void insert(pair<int, int> key, const Entry &entry);
    Entry remove_min();
    Entry remove_rand_node(MTRand_int32 *rand_gen);
    void clear();

    bool empty() const;
    
    size_t approx_num_bytes() const;
    int size() const;
};

#include "open_list.cc"

// HACK! Need a better strategy of dealing with templates, also in the Makefile.

#endif
