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
        //初始化马尔科夫决策过程
        MDP simulation = MDP(true, "");
        vector<pair<Aggregation, double> > valueAtThisSimulation;
        //开始mdp模拟
        while (simulation.currentState.currentRoute != nullptr)
        {
            Action bestAction;
            double reward = 0.0;
            simulation.findBestAction(&bestAction, *valueFunction, &reward, true);
            Aggregation postDecisionState;
            simulation.currentState.executeAction(bestAction);
            postDecisionState.aggregate(simulation.currentState, bestAction);
            simulation.currentState.undoAction(bestAction);
            //记录这次sample path的信息
            valueAtThisSimulation.push_back(make_pair(postDecisionState, reward));
            //状态转移
            simulation.transition(bestAction);
        }
        //对lookup table 进行更新
        simulation.solution.calcCost();
        //cout << totalSimulationCount << " " << simulation.solution.cost << " " << simulation.solution.penalty << " " << simulation.solution.waitTime << endl;
        valueFunction->updateValue(valueAtThisSimulation, startApproximate);
        for (auto iter = simulation.customers.begin(); iter != simulation.customers.end(); ++iter)
        {
            delete iter->second;
        }
    }
}
