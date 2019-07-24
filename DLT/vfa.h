#pragma once
#include "avi.h"
#include "customer.h"
#include "route.h"
#include "util.h"
//#include "Eigen/Dense"
#include <vector>
#define STEP_SIZE 0.3
#define PARTITION_THRESHOLD 1.5
#define X_INITIAL_ENTRY_NUM 10.0
#define Y_INITIAL_ENTRY_NUM 10.0
#define DYNAMIC_LOOKUP_TABLE 1
#define ATTRIBUTES_NUMBER 4
#define LAMBDA 1

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
  map<int, double> entryValue;
  int entryIndex[int(MAX_WORK_TIME)][int(MAX_VEHICLE * MAX_WORK_TIME)];
  map<int, pair<double, double> > entryPosition;
  map<int, pair<double, double> > entryRange;
  map<int, pair<int, vector<double> > > entryInfo;
  double lookup(Aggregation postDecisionState);
  void partitionUpdate();
  void partition(int entryNum);
  LookupTable();
};
class ValueFunction
{
/*public:
  LookupTable lookupTable;
  double lambda;
  Eigen::Matrix4d matrixBeta;
  Eigen::Vector4d attributesWeight;
  //double getValue(Aggregation postDecisionState, double reward);
  double getValue(State S, Action a);
  //void updateValue(vector<pair<Aggregation, double>> valueAtThisSimulation, bool startApproximate);
  void updateValue(vector<pair<Eigen::Vector4d, double> > valueAtThisSimulation, bool startApproximate);
  ValueFunction();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW*/
public:
  LookupTable lookupTable;
  double getValue(State S, Action a, bool approx);
  void updateValue(vector<pair<Aggregation, double> > valueAtThisSimulation, bool startApproximate);
  ValueFunction();
};
