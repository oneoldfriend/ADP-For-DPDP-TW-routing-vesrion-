#include "avi.h"
#include "generator.h"
#include "vfa.h"
#include <random>

void AVI::approximation(ValueFunction *valueFunction)
{
    //定义计数器，包括总模拟次数和每个instance的模拟次数
    int totalSimulationCount = 0;
    int lagApproximateCount = 0;
    bool startApproximate = false;
    while (totalSimulationCount++ < MAX_SIMULATION)
    {
        double duration;
        clock_t start, end;
        start = clock();
        lagApproximateCount++;
        if (lagApproximateCount > LAG_APPROXIMATE)
        {
            startApproximate = true;
        }
        //初始化马尔科夫决策过程
        MDP simulation = MDP(true, "");
        vector<pair<Aggregation, double>> valueAtThisSimulation;
        //开始mdp模拟
        while (simulation.currentState.currentRoute != nullptr)
        {
            Action bestAction;
            double routingReward = 0.0;
            simulation.findBestAssignmentAction(&bestAction, valueFunction);
            simulation.assignmentConfirmed(bestAction);
            simulation.findBestRoutingAction(&bestAction, valueFunction, &routingReward, startApproximate);
            //记录这次sample path的信息
            simulation.executeAction(bestAction);
            Aggregation postDecisionState;
            postDecisionState.aggregate(simulation.currentState, bestAction);
            simulation.undoAction(bestAction);
            valueAtThisSimulation.push_back(make_pair(postDecisionState, routingReward));
            //状态转移
            simulation.transition(bestAction);
        }
        //对lookup table 进行更新
        //cout << totalSimulationCount << " " << simulation.solution.cost << " " << simulation.solution.penalty << " " << simulation.solution.waitTime << " " << simulation.cumOutsourcedCost << " " << simulation.solution.cost + simulation.cumOutsourcedCost << " " << valueSum << endl;
        valueFunction->updateValue(valueAtThisSimulation, startApproximate);
        for (auto iter = simulation.customers.begin(); iter != simulation.customers.end(); ++iter)
        {
            delete iter->second;
        }
        end = clock();
        duration = (double)(end - start) / CLOCKS_PER_SEC;
        //cout << "last " << duration << "s  " << valueAtThisSimulation.size() << endl;
    }
}
