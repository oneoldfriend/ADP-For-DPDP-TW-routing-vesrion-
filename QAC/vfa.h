#pragma once
#include "avi.h"
#include "customer.h"
#include "route.h"
#include "util.h"
#include "Eigen/Dense"
#include <vector>
#define PARTITION_THRESHOLD 1.0
#define LOOKUP_TABLE_INITIAL 10.0
#define ATTRIBUTES_NUMBER 4
#define STEP_SIZE 0.3
#define LAMBDA 0.8
#define GAMMA 0.98
#define BETA 0.01

class ValueFunction
{
public:
  Eigen::Vector4d actorWeights;
  Eigen::Vector4d criticWeights;
  Eigen::Matrix4d matrixBeta;
  double getValue(State S, Action a, bool approx);
  void updateActor(pair<Eigen::Vector4d, double> infoAtCurrentState, State nextState, Action actionForNextState, Eigen::Vector4d score);
  void updateCritic(vector<pair<Eigen::Vector4d, double> > valueAtThisSimulation, bool startApproximate);
  ValueFunction();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
