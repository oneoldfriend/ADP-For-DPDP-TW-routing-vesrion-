#pragma once
#include "Eigen/Dense"
#include <vector>

using namespace std;

class Action;
class Route;
class Solution
{
public:
  vector<Route> routes;
  Eigen::Vector4d info;
  double cost, penalty, waitTime, travelTime;
  bool greedyInsertion(Action a);
  void solutionCopy(Solution *source);
  void solutionDelete();
  double calcCost();
  void calcInfo();
  Solution();
};