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

class Action
{
public:
  map<Customer *, bool> customerConfirmation;
  double rejectionReward();
};

class State
{
public:
  Solution *pointSolution;
  double currentTime;
  vector<string> notServicedCustomer;
  vector<string> newCustomers;
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
  void findBestAction(Action *a, ValueFunction valueFunction, double *value);
  void integerToAction(int actionNum, State S, Action *a);
  void transition(Action a);
  double reward(State S, Action a);
  void observation(double lastDecisionTime);
  MDP(string fileName);
};