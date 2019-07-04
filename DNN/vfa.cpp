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
    S.calcAttribute(a);
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
    nextState.calcAttribute(actionForNextState);
    double estimateValue = this->criticWeights.transpose() * infoAtCurrentState.first;
    double bootstrappingValue = infoAtCurrentState.second + LAMBDA * this->criticWeights.transpose() * nextState.attributes;
    double error = bootstrappingValue - estimateValue;
    this->actorWeights = this->actorWeights + STEP_SIZE * score * error;
    double gammaN = 1.0 + infoAtCurrentState.first.transpose() * this->matrixBeta * infoAtCurrentState.first;
    this->criticWeights = this->criticWeights + 1 / gammaN * this->matrixBeta * infoAtCurrentState.first * error;
    this->matrixBeta = this->matrixBeta - 1.0 / gammaN * (this->matrixBeta * infoAtCurrentState.first * infoAtCurrentState.first.transpose() * this->matrixBeta);
    nextState.undoAction(actionForNextState);
}

void ValueFunction::updateCritic(vector<pair<Eigen::Vector4d, double>> valueAtThisSimulation, bool startApproximate, vector<Eigen::Vector4d> scoreAtThisSimulation)
{
    double lastValue = 0;
    for (auto iter = valueAtThisSimulation.rbegin(); iter != valueAtThisSimulation.rend(); ++iter)
    {
        iter->second += lastValue;
        lastValue = double(LAMBDA) * iter->second;
    }
    auto scoreIter = scoreAtThisSimulation.begin();
    for (auto iter = valueAtThisSimulation.begin(); iter != valueAtThisSimulation.end(); ++iter, ++scoreIter)
    {
        double gammaN = 1.0 + iter->first.transpose() * this->matrixBeta * iter->first,
               error = this->criticWeights.transpose() * iter->first - iter->second;
        this->criticWeights = this->criticWeights - 1 / gammaN * this->matrixBeta * iter->first * error;
        this->matrixBeta = this->matrixBeta - 1.0 / gammaN * (this->matrixBeta * iter->first * iter->first.transpose() * this->matrixBeta);
    }
}
