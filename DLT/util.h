#pragma once
#include"route.h"
#include <math.h>
#include <vector>
#include <iostream>

using namespace std;

class Util
{
  public:
    static void infoCopy(Customer *target, Customer *source);
    static double standardDeviation(double *sample, int size);
    static double calcTravelTime(Position a, Position b);
};