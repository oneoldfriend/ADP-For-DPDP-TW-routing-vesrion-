#include "vfa.h"
#include <iomanip>

ValueFunction::ValueFunction()
{
    for (int i = 0; i < ATTRIBUTES_NUMBER; i++)
    {
        this->actorWeights[i] = 1.0;
        this->criticWeights[i] = 1.0;
    }
    this->matrixBeta = MAX_EDGE * CUSTOMER_NUMBER / MAX_VEHICLE * Eigen::Matrix4d::Identity();
}

double ValueFunction::getValue(State S, Action a, bool actor)
{
    S.calcAttribute();
    if (MYOPIC)
    {
        return 0;
    }
    else
    {
        if (actor)
        {
            return this->actorWeights.transpose() * S.attributes;
        }
        else
        {
            return this->criticWeights.transpose() * S.attributes;
        }
    }
}

void ValueFunction::updateActor(pair<Eigen::Vector4d, double> infoAtCurrentState, State nextState, Action actionForNextState, Eigen::Vector4d score)
{
    nextState.executeAction(actionForNextState);
    nextState.calcAttribute();
    double estimateValueForCurrentState = this->criticWeights.transpose() * infoAtCurrentState.first;
    this->actorWeights = this->actorWeights - STEP_SIZE * score * estimateValueForCurrentState;
    nextState.undoAction(actionForNextState);
}


void ValueFunction::updateCritic(vector<pair<Eigen::Vector4d, double>> valueAtThisSimulation, bool startApproximate)
{
    double lastValue = 0;
    double weightErrorThisSimulation = 0.0, valueErrorThisSimulation = 0.0;
    for (auto iter = valueAtThisSimulation.rbegin(); iter != valueAtThisSimulation.rend(); ++iter)
    {
        iter->second += lastValue;
        lastValue = double(LAMBDA) * iter->second;
    }
    Eigen::Vector4d oldAttributesWeight = this->criticWeights;
    for (auto iter = valueAtThisSimulation.begin(); iter != valueAtThisSimulation.end(); ++iter)
    {
        double gammaN = 1.0 + iter->first.transpose() * this->matrixBeta * iter->first,
               error = this->criticWeights.transpose() * iter->first - iter->second;
        this->criticWeights = this->criticWeights - 1 / gammaN * this->matrixBeta * iter->first * error;
        this->matrixBeta = this->matrixBeta - 1.0 / gammaN * (this->matrixBeta * iter->first * iter->first.transpose() * this->matrixBeta);
        valueErrorThisSimulation += abs(error);
    }
    for (int i = 0; i < ATTRIBUTES_NUMBER; i++)
    {
        weightErrorThisSimulation += abs(this->criticWeights[i] - oldAttributesWeight[i]);
    }
}

