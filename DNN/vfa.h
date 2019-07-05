#pragma once
#include "avi.h"
#include "customer.h"
#include "route.h"
#include "util.h"
#include "Eigen/Dense"
#include "mxnet-cpp/MxNetCpp.h"
#include "mxnet-cpp/model.h"
#include "mxnet-cpp/op.h"
#include <vector>
#define PARTITION_THRESHOLD 1.0
#define LOOKUP_TABLE_INITIAL 10.0
#define ATTRIBUTES_NUMBER 4
#define STEP_SIZE 0.2
#define LAMBDA 0.8


using namespace std;
using namespace mxnet::cpp;

class ValueFunction
{
public:
  Symbol net;
  Eigen::Vector4d actorWeights;
  Eigen::Vector4d criticWeights;
  Eigen::Matrix4d matrixBeta;
  double getValue(State S, Action a, bool approx);
  void updateActor(pair<Eigen::Vector4d, double> infoAtCurrentState, State nextState, Action actionForNextState, Eigen::Vector4d score);
  void updateCritic(vector<pair<Eigen::Vector4d, double> > valueAtThisSimulation, bool startApproximate, vector<Eigen::Vector4d> scoreAtThisSimulation);
  ValueFunction(const vector<int> &layers);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
