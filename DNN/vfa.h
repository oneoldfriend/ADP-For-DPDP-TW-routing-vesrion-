#pragma once
#include "avi.h"
#include "customer.h"
#include "route.h"
#include "util.h"
#include "mxnet-cpp/MxNetCpp.h"
#include "mxnet-cpp/model.h"
#include "mxnet-cpp/op.h"
#include <vector>
#define FIRST_LAYER 512
#define SECOND_LAYER 10
#define INPUT_DATA_FIRST_D 100
#define INPUT_DATA_SECOND_D 1
#define PARTITION_THRESHOLD 1.0
#define LOOKUP_TABLE_INITIAL 10.0
#define ATTRIBUTES_NUMBER 4
#define STEP_SIZE 0.2
#define LAMBDA 0.8

class State;
class Action;

using namespace std;
using namespace mxnet::cpp;

class ValueFunction
{
public:
  vector<Symbol> weights;
  vector<Symbol> biases;
  vector<Symbol> outputs;
  Symbol net;
  Executor *exe;
  double getValue(State S, Action a, bool approx);
  void updateNetwork(double valueAtThisSimulation[(int)MAX_WORK_TIME][INPUT_DATA_FIRST_D][INPUT_DATA_SECOND_D], vector<double> rewardPath, bool startApproximate);
  ValueFunction(const vector<int> &layers);
};
