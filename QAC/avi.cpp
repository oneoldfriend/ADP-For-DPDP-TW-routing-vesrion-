#include "avi.h"
#include "generator.h"
#include <random>

void AVI::approximation(ValueFunction *valueFunction)
{
    //定义计数器，包括总模拟次数和每个instance的模拟次数
    int totalSimulationCount = 0;
    int lagApproximateCount = 0;
    bool startApproximate = false;
    while (totalSimulationCount++ < MAX_SIMULATION)
    {
        lagApproximateCount++;
        if (lagApproximateCount > LAG_APPROXIMATE)
        {
            startApproximate = true;
        }
        double valueSum = 0.0;
        //初始化马尔科夫决策过程
        MDP simulation = MDP(true, "");
        pair<Eigen::Vector4d, double> infoAtCurrentState;
        Eigen::Vector4d currentScore;
        Eigen::Vector4d nextScore;
        //单步初始化
        Action bestAction;
        double routingReward = 0.0;
        simulation.findBestAssignmentAction(&bestAction, *valueFunction);
        simulation.assignmentConfirmed(bestAction);
        simulation.findBestRoutingAction(&bestAction, *valueFunction, &routingReward, startApproximate, &currentScore);
        cout << currentScore << endl;
        //开始mdp模拟
        while (simulation.currentState.currentRoute != nullptr)
        {
            valueSum += routingReward;
            //记录这次sample path的信息
            simulation.currentState.executeAction(bestAction);
            simulation.currentState.calcAttribute();
            simulation.currentState.undoAction(bestAction);
            infoAtCurrentState = make_pair(simulation.currentState.attributes, routingReward);
            //状态转移
            simulation.transition(bestAction);
            //对下一次状态采样
            simulation.findBestAssignmentAction(&bestAction, *valueFunction);
            simulation.assignmentConfirmed(bestAction);
            simulation.findBestRoutingAction(&bestAction, *valueFunction, &routingReward, startApproximate, &nextScore);
            valueFunction->updateValue(infoAtCurrentState, simulation.currentState, bestAction, currentScore);
            currentScore = nextScore;
        }
        simulation.solution.calcCost();
        //cout << totalSimulationCount << " " << simulation.solution.cost << " " << simulation.solution.penalty << " " << simulation.solution.waitTime << " " << simulation.cumOutsourcedCost << " " << simulation.solution.cost + simulation.cumOutsourcedCost << " " << valueSum << endl;
        for (auto iter = simulation.customers.begin(); iter != simulation.customers.end(); ++iter)
        {
            delete iter->second;
        }
    }
}
