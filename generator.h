#pragma once
#include "avi.h"

class Generator
{
public:
    static void instanceGenenrator(double trainDayNum);
    static bool sortAscend(const pair<double, Customer *> a, const pair<double, Customer *> b);
};