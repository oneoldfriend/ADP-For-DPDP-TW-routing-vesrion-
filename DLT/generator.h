#pragma once
#include "avi.h"
#include <random>

class Generator
{
public:
    static void instanceGenenrator(list<pair<double, Customer *> > *sequenceData);
    static bool sortAscend(const pair<double, Customer *> a, const pair<double, Customer *> b);
};