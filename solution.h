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
  Eigen::Vector4d attribute;
  double cost;
  bool greedyInsertion(Action a);
  void solutionCopy(Solution *source);
  void solutionDelete();
  double calcCost();
  void calcAttribute();
  Solution();
};