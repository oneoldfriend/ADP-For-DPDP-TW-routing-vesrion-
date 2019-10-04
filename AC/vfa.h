#pragma once
#include "avi.h"
#include "customer.h"
#include "route.h"
#include "util.h"
#include "Eigen/Dense"
#include <vector>
#define PARTITION_THRESHOLD 1.0
#define LOOKUP_TABLE_INITIAL 10.0
#define ATTRIBUTES_NUMBER 5
#define LAMBDA 1.0
#define NOISE_DEDUCTION 0.8
#define ALPHA 1e-6

class ValueFunction
{
public:
  Eigen::MatrixXd routingMatrixBeta;
  Eigen::MatrixXd assignmentMatrixBeta;
  Eigen::VectorXd routingCriticWeight;
  Eigen::VectorXd assignmentCriticWeight;
  Eigen::VectorXd routingActorWeight;
  Eigen::VectorXd assignmentActorWeight;
  double getValue(State S, Action a, bool assignment, bool myopic);
  void criticUpdate(vector<pair<Eigen::VectorXd, double> > routingValueAtThisSimulation, vector<pair<Eigen::VectorXd, double> > assignmentValueAtThisSimulation, bool startInteraction);
  void actorUpdate(vector<pair<Eigen::VectorXd, pair<Eigen::VectorXd, double>>> routingReplayBuffer, vector<pair<Eigen::VectorXd, pair<Eigen::VectorXd, double>>> assignmentReplayBuffer);
  ValueFunction();
  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
