#pragma once
//#include "Eigen/Dense"
#include <vector>

using namespace std;

class Action;
class Route;
class Solution
{
public:
  vector<Route> routes;
  double cost, penalty, waitTime, travelTime;
  void solutionCopy(Solution *source);
  void solutionDelete();
  double calcCost();
  //Eigen::Vector4d attribute;
  //void calcAttribute();
  //bool greedyInsertion(Action a);
  Solution();
};