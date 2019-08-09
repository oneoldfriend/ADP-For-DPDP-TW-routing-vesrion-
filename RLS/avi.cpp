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
        clock_t start, end;
        start = clock();
        lagApproximateCount++;
        if (lagApproximateCount > LAG_APPROXIMATE)
        {
            startApproximate = true;
        }
        //初始化马尔科夫决策过程
        MDP simulation = MDP(true, "");
        vector<pair<Eigen::VectorXd, double>> routingValueAtThisSimulation;
        vector<pair<Eigen::VectorXd, double>> assignmentValueAtThisSimulation;
        //开始mdp模拟
        while (simulation.currentState.currentRoute != nullptr)
        {
            Action bestAction;
            double routingReward = 0.0, assignmentReward = 0.0;
            simulation.findBestAssignmentAction(&bestAction, *valueFunction, &assignmentReward);
            simulation.currentState.calcAttribute(bestAction, true);
            assignmentValueAtThisSimulation.push_back(make_pair(simulation.currentState.attributes, assignmentReward));
            simulation.assignmentConfirmed(bestAction);
            simulation.findBestRoutingAction(&bestAction, *valueFunction, &routingReward);
            //记录这次sample path的信息
            simulation.executeAction(bestAction);
            simulation.currentState.calcAttribute(bestAction, false);
            simulation.undoAction(bestAction);
            routingValueAtThisSimulation.push_back(make_pair(simulation.currentState.attributes, routingReward));
            //状态转移
            simulation.transition(bestAction);
        }
        //对lookup table 进行更新
        double valueSum = 0.0;
        for (auto iter = routingValueAtThisSimulation.begin(); iter != routingValueAtThisSimulation.end(); ++iter)
        {
            valueSum += iter->second;
        }
        simulation.solution.calcCost();
        //cout << totalSimulationCount << " " << simulation.solution.cost << " " << simulation.solution.penalty << " " << simulation.solution.waitTime << " " << simulation.cumOutsourcedCost << " " << simulation.solution.cost + simulation.cumOutsourcedCost << " " << valueSum << endl;
        valueFunction->updateValue(routingValueAtThisSimulation, assignmentValueAtThisSimulation, startApproximate);
        for (auto iter = simulation.customers.begin(); iter != simulation.customers.end(); ++iter)
        {
            delete iter->second;
        }
        end = clock();
        cout << double(end - start) / CLOCKS_PER_SEC << endl;
    }
}
