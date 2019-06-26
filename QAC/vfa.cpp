#include "vfa.h"
#include <iomanip>

ValueFunction::ValueFunction()
{
    for (int i = 0; i < ATTRIBUTES_NUMBER; i++)
    {
        this->actorWeights[i] = 1.0;
        this->criticWeights[i] = 1.0;
    }
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

void ValueFunction::updateValue(pair<Eigen::Vector4d, double> infoAtCurrentState, State nextState, Action actionForNextState, Eigen::Vector4d score)
{
    nextState.executeAction(actionForNextState);
    nextState.calcAttribute();
    double estimateValueForCurrentState = this->criticWeights.transpose() * infoAtCurrentState.first;
    double error = infoAtCurrentState.second + GAMMA * this->getValue(nextState, actionForNextState, false) - estimateValueForCurrentState;
    this->actorWeights = this->actorWeights + STEP_SIZE * score * estimateValueForCurrentState;
    this->criticWeights = this->criticWeights + BETA * error * infoAtCurrentState.first;
}
