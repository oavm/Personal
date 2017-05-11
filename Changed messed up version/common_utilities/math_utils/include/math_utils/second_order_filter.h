/*
 * Utilities.h
 *
 *  Created on: Sept 23, 2015
 *      Author: Felipe Polido
 *
 */
#ifndef SECOND_ORDER_FILTER_H_
#define SECOND_ORDER_FILTER_H_

template <typename T>
inline T secondOrderFilter(T & varOutputSecondFilter , T & varOutputFirstFilter , T const& varNew , T const& gain)
{ 
    varOutputFirstFilter = (1- gain) * varOutputFirstFilter + gain * varNew;
    varOutputSecondFilter = (1 - gain) * varOutputSecondFilter + gain * varOutputFirstFilter;
    return varOutputSecondFilter;
} 

#endif
