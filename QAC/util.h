#pragma once
#include"route.h"
#include <math.h>
#include <vector>
#include <iostream>
#include <map>
#include "Eigen/Dense"

using namespace std;

class Util
{
  public:
    static void infoCopy(Customer *target, Customer *source);
    static double standardDeviation(vector<double> sample);
    static double calcTravelTime(Position a, Position b);
    static int softmax(map<int, double> data, Eigen::Vector4d *scoreExpectation, map<int, Eigen::Vector4d> actionScoreSample);
};