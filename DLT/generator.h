#pragma once
#include "avi.h"
#include <random>

class Generator
{
public:
    static void instanceGenenrator(string fileName);
    static bool sortAscend(const pair<double, Customer *> a, const pair<double, Customer *> b);
};