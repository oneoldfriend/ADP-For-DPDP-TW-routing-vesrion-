#pragma once
#include "customer.h"
#include "solution.h"
#include <map>
#include <list>
#include <string>
#include <vector>
#include <fstream>

using namespace std;

class ValueFunction;
typedef class Order *PointOrder;

class Action
{
public:
  PointOrder positionToVisit;
};

class State
{
public:
  Route *currentRoute;
  double currentTime;
  Solution *pointSolution;
  vector<PointOrder> notServicedCustomer;
  vector<PointOrder> reachableCustomer;
  void executeAction(Action a);
  void undoAction(Action a);
  State();
};

class MDP
{
public:
  Solution solution;
  double cumRejectionReward;
  State currentState;
  list<pair<double, Customer *> > sequenceData;
  map<string, Customer*> customers;
  bool checkActionFeasibility(Action a, double *reward);
  void findBestAction(Action *a, ValueFunction valueFunction, double *reward, bool approx);
  void integerToAction(int actionNum, State S, Action *a);
  void transition(Action a);
  double reward(State S, Action a);
  void observation(double lastDecisionTime);
  MDP(string fileName);
};