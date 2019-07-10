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
  pair<PointOrder, PointOrder> destPosBeforeExecution;
  map<Customer *, bool> customerConfirmation;
  double rejectionReward();
};

class State
{
public:
  Route *currentRoute;
  double currentTime;
  Solution *pointSolution;
  Eigen::VectorXd attributes;
  vector<Customer*> newCustomers;
  map<string, pair<PointOrder, PointOrder> > notServicedCustomer;
  vector<PointOrder> reachableCustomer;
  void calcAttribute(Action a);
  State();
};

class MDP
{
public:
  Solution solution;
  double cumOutsourcedCost;
  State currentState;
  list<pair<double, Customer *> > sequenceData;
  map<string, Customer*> customers;
  bool checkAssignmentActionFeasibility(Action a, double *reward);
  bool checkRoutingActionFeasibility(Action a, double *reward);
  void findBestAssignmentAction(Action *a, ValueFunction valueFunction);
  void findBestRoutingAction(Action *a, ValueFunction valueFunction, double *reward, bool approx);
  void integerToRoutingAction(int actionNum, State S, Action *a);
  void integerToAssignmentAction(int actionNum, State S, Action *a);
  void transition(Action a);
  void checkIgnorance(Action a);
  void assignmentConfirmed(Action a);
  double reward(State S, Action a);
  void observation(double lastDecisionTime);
  void executeAction(Action a);
  void undoAction(Action a);
  MDP(bool approx, string fileName);
  ~MDP();
};