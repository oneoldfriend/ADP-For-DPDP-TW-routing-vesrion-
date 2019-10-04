#include "vfa.h"
#include <iomanip>

ValueFunction::ValueFunction()
{
    this->routingCriticWeight = Eigen::VectorXd(ATTRIBUTES_NUMBER);
    this->assignmentCriticWeight = Eigen::VectorXd(ATTRIBUTES_NUMBER);
    this->routingActorWeight = Eigen::VectorXd(ATTRIBUTES_NUMBER);
    this->assignmentActorWeight = Eigen::VectorXd(ATTRIBUTES_NUMBER);
    this->routingMatrixBeta = Eigen::MatrixXd(ATTRIBUTES_NUMBER, ATTRIBUTES_NUMBER);
    this->assignmentMatrixBeta = Eigen::MatrixXd(ATTRIBUTES_NUMBER, ATTRIBUTES_NUMBER);
    for (int i = 0; i < ATTRIBUTES_NUMBER; i++)
    {
        this->routingCriticWeight(i) = 1.0;
        this->assignmentCriticWeight(i) = 1.0;
        this->routingActorWeight(i) = 1.0;
        this->assignmentActorWeight(i) = 1.0;
        for (int j = 0; j < ATTRIBUTES_NUMBER; j++)
        {
            if (i == j)
            {
                this->routingMatrixBeta(i, j) = MAX_EDGE * CUSTOMER_NUMBER / MAX_VEHICLE;
                this->assignmentMatrixBeta(i, j) = MAX_EDGE * CUSTOMER_NUMBER / MAX_VEHICLE;
            }
            else
            {
                this->routingMatrixBeta(i, j) = 0;
                this->assignmentMatrixBeta(i, j) = 0;
            }
        }
    }
}


double ValueFunction::getValue(State S, Action a, bool assignment, bool myopic)
{
    S.calcAttribute(a, assignment);
    if (assignment)
    {
        if (ASSIGNMENT_MYOPIC || myopic)
        {
            return 0;
        }
        else
        {
            return this->assignmentActorWeight.transpose() * S.attributes;
        }
    }
    else
    {
        if (ROUTING_MYOPIC || myopic)
        {
            return 0;
        }
        else
        {
            return this->routingActorWeight.transpose() * S.attributes;
        }
    }
}

void ValueFunction::criticUpdate(vector<pair<Eigen::VectorXd, double>> routingValueAtThisSimulation, vector<pair<Eigen::VectorXd, double>> assignmentValueAtThisSimulation, bool startInteraction)
{
    double lastValue = 0.0;
    for (int i = 0; i < (int)assignmentValueAtThisSimulation.size(); i++)
    {
        routingValueAtThisSimulation[i].second += assignmentValueAtThisSimulation[i].second;
    }
    for (auto iter = routingValueAtThisSimulation.rbegin(); iter != routingValueAtThisSimulation.rend(); ++iter)
    {
        iter->second += lastValue;
        lastValue = double(NOISE_DEDUCTION) * iter->second;
    }
    for (int i = 0; i < (int)assignmentValueAtThisSimulation.size(); i++)
    {
        assignmentValueAtThisSimulation[i].second = routingValueAtThisSimulation[i].second;
    }

    for (int i = 0; i < (int)routingValueAtThisSimulation.size(); i++)
    {
        double gammaNForRouting = LAMBDA + routingValueAtThisSimulation[i].first.transpose() * this->routingMatrixBeta * routingValueAtThisSimulation[i].first,
               gammaNForAssignment = LAMBDA + assignmentValueAtThisSimulation[i].first.transpose() * this->assignmentMatrixBeta * assignmentValueAtThisSimulation[i].first,
               errorForRouting = 0.0, errorForAssignment = 0.0,
               estimationErrorForRouting = this->routingCriticWeight.transpose() * routingValueAtThisSimulation[i].first - 0.0 - this->assignmentCriticWeight.transpose() * assignmentValueAtThisSimulation[i].first,
               estimationErrorForAssignment = this->assignmentCriticWeight.transpose() * assignmentValueAtThisSimulation[i].first - 0.0 - this->routingCriticWeight.transpose() * routingValueAtThisSimulation[i].first;

        if (!ROUTING_MYOPIC)
        {
            errorForRouting += this->routingCriticWeight.transpose() * routingValueAtThisSimulation[i].first - routingValueAtThisSimulation[i].second;
        }
        if (!ASSIGNMENT_MYOPIC)
        {
            errorForAssignment += this->assignmentCriticWeight.transpose() * assignmentValueAtThisSimulation[i].first - assignmentValueAtThisSimulation[i].second;
        }

        double ratioForRouting = estimationErrorForRouting / errorForRouting, ratioForAssignment = estimationErrorForAssignment / errorForAssignment;

        //error iteraction
        if (startInteraction && !ASSIGNMENT_MYOPIC && !ROUTING_MYOPIC)
        {
            errorForRouting += errorForAssignment;
            errorForAssignment = errorForRouting;
        }

        this->routingCriticWeight = this->routingCriticWeight - 1.0 / gammaNForRouting * this->routingMatrixBeta * routingValueAtThisSimulation[i].first * errorForRouting;
        this->routingMatrixBeta = LAMBDA * (this->routingMatrixBeta - 1.0 / gammaNForRouting * (this->routingMatrixBeta * routingValueAtThisSimulation[i].first * routingValueAtThisSimulation[i].first.transpose() * this->routingMatrixBeta));

        this->assignmentCriticWeight = this->assignmentCriticWeight - 1.0 / gammaNForAssignment * this->assignmentMatrixBeta * assignmentValueAtThisSimulation[i].first * errorForAssignment;
        this->assignmentMatrixBeta = LAMBDA * (this->assignmentMatrixBeta - 1.0 / gammaNForAssignment * (this->assignmentMatrixBeta * assignmentValueAtThisSimulation[i].first * assignmentValueAtThisSimulation[i].first.transpose() * this->assignmentMatrixBeta));
    }
}

void ValueFunction::actorUpdate(vector<pair<Eigen::VectorXd, pair<Eigen::VectorXd, double>>> routingReplayBuffer, vector<pair<Eigen::VectorXd, pair<Eigen::VectorXd, double>>> assignmentReplayBuffer)
{
    for (int i = 0; i < (int)routingReplayBuffer.size(); i++)
    {
        double routingTDError = routingReplayBuffer[i].second.second + NOISE_DEDUCTION * this->routingCriticWeight.transpose() * routingReplayBuffer[i].second.first - this->routingCriticWeight.transpose() * routingReplayBuffer[i].first;
        double assignmentTDError = assignmentReplayBuffer[i].second.second + NOISE_DEDUCTION * this->assignmentCriticWeight.transpose() * assignmentReplayBuffer[i].second.first - this->assignmentCriticWeight.transpose() * assignmentReplayBuffer[i].first;

        if (!ROUTING_MYOPIC)
        {
            this->routingActorWeight = this->routingActorWeight + ALPHA * routingTDError * routingReplayBuffer[i].first;
        }
        if (!ASSIGNMENT_MYOPIC)
        {
            this->assignmentActorWeight = this->assignmentActorWeight + ALPHA * assignmentTDError * assignmentReplayBuffer[i].first;
        }
    }
}
