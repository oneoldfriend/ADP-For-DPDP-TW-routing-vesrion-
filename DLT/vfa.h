#pragma once
#include "avi.h"
#include "customer.h"
#include "route.h"
#include "util.h"
#include <vector>
#define STEP_SIZE 0.3
#define PARTITION_THRESHOLD 2.0
#define DYNAMIC_LOOKUP_TABLE 0
#define ATTRIBUTES_NUMBER 4
#define LAMBDA 0.8

class Aggregation
{
public:
  double currentTime, remainTime;
  void aggregate(State S, Action a);
  Aggregation();
};

class LookupTable
{
public:
  double averageN, averageTheta;
  map<int, double> entryValue;
  vector<vector<int> > entryIndex;
  map<int, pair<double, double> > entryPosition;
  map<int, pair<double, double> > entryRange;
  map<int, double[3]> entryInfo;
  double lookup(Aggregation postDecisionState);
  void partitionCheck();
  void partition(int entryNum);
  LookupTable();
};
class ValueFunction
{
public:
  LookupTable lookupTable;
  double getValue(State S, Action a, bool approx);
  void updateValue(vector<pair<Aggregation, double> > valueAtThisSimulation, bool startApproximate);
  ValueFunction();
};
