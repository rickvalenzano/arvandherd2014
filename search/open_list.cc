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

// HACK! Ignore this if used as a top-level compile target.
#ifdef OPEN_LIST_H
#define OL_ENTRY_OVERHEAD 0
#define OL_LOAD_FACTOR 1

#include <iostream>
using namespace std;

/*
  Priority_queue based implementation of an open list.
*/

template<class Entry>
OpenList<Entry>::OpenList() {
}

template<class Entry>
OpenList<Entry>::~OpenList() {
}

template<class Entry>
void OpenList<Entry>::insert(pair<int, int> key, const Entry &entry) {
	the_heap.push_back(make_pair(key, entry));
	HeapifyUp(the_heap.size()-1); // heapify from back of the heap
}

template<class Entry>
Entry OpenList<Entry>::remove_min() {
	Entry ans = the_heap[0].second;
	the_heap[0] = the_heap[the_heap.size()-1];
	the_heap.pop_back();
	HeapifyDown(0);

	return ans;
}

template<class Entry>
Entry OpenList<Entry>::remove_rand_node(MTRand_int32 *rand_gen) {
	int rand_num = rand_gen->get_32bit_int() % the_heap.size();
	Entry ans = the_heap[rand_num].second;
	the_heap[rand_num] = the_heap[the_heap.size()-1];
	the_heap.pop_back();

	if(!HeapifyDown(rand_num))
		HeapifyUp(rand_num);

	return ans;
}

template<class Entry>
inline bool OpenList<Entry>::empty() const {
    return the_heap.empty();
}

template<class Entry>
inline void OpenList<Entry>::clear() {
	the_heap.clear();
}

template<class Entry>
size_t OpenList<Entry>::approx_num_bytes() const {
    size_t value = sizeof(the_heap) +
                   OL_LOAD_FACTOR*the_heap.capacity()*(OL_ENTRY_OVERHEAD + sizeof(pair<pair<int, int>, Entry>));
    return value;
}

template<class Entry>
int OpenList<Entry>::size() const {
    return the_heap.size();
}

template<class Entry>
bool OpenList<Entry>::HeapifyUp(unsigned int index)
{
	if (index == 0) return false;
	int parent = (index-1)/2;
	IsWorse compare;

	if (compare(the_heap[parent], the_heap[index]))
	{
		pair<pair<int,int>, Entry> tmp = the_heap[parent];
		the_heap[parent] = the_heap[index];
		the_heap[index] = tmp;
		HeapifyUp(parent);
		return true;
	}
	return false;
}

template<class Entry>
bool OpenList<Entry>::HeapifyDown(unsigned int index)
{
	IsWorse compare;
	unsigned int child1 = index*2+1;
	unsigned int child2 = index*2+2;
	int which;
	unsigned int count = the_heap.size();
	// find smallest child
	if (child1 >= count)
		return false;
	else if (child2 >= count)
		which = child1;
	else if (!(compare(the_heap[child1], the_heap[child2])))
		which = child1;
	else
		which = child2;

	if (!(compare(the_heap[which], the_heap[index])))
	{
		pair<pair<int,int>, Entry> tmp = the_heap[which];
		the_heap[which] = the_heap[index];

		the_heap[index] = tmp;

		HeapifyDown(which);
		return true;
	}
	return false;
}
#endif
