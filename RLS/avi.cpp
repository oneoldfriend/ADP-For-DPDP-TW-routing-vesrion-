#include "avi.h"
#include "generator.h"
#include <random>

void AVI::approximation(ValueFunction *valueFunction)
{
    //定义计数器，包括总模拟次数和每个instance的模拟次数
    int totalSimulationCount = 0, isSwitch = 0, switchCount = 0;
    list<pair<double, Customer *>> data;
    Generator::instanceGenenrator(false, &data, "");
    vector<pair<Eigen::VectorXd, double>> routingValueAtThisSimulation;
    vector<pair<Eigen::VectorXd, double>> assignmentValueAtThisSimulation;
    while (totalSimulationCount < MAX_SIMULATION)
    {
        clock_t start, end;
        start = clock();

        //初始化马尔科夫决策过程
        MDP simulation = MDP(true, "", &data);
        //开始mdp模拟
        while (simulation.currentState.currentRoute != nullptr)
        {
            Action bestAction;
            double routingReward = 0.0, assignmentReward = 0.0;
            simulation.findBestAssignmentAction(&bestAction, *valueFunction, &assignmentReward, false);
            simulation.currentState.calcAttribute(bestAction, true);
            assignmentValueAtThisSimulation.push_back(make_pair(simulation.currentState.attributes, assignmentReward));
            simulation.assignmentConfirmed(bestAction);
            simulation.findBestRoutingAction(&bestAction, *valueFunction, &routingReward, false);
            //记录这次sample path的信息
            simulation.executeAction(bestAction);
            simulation.currentState.calcAttribute(bestAction, false);
            simulation.undoAction(bestAction);
            routingValueAtThisSimulation.push_back(make_pair(simulation.currentState.attributes, routingReward));
            //状态转移
            simulation.transition(bestAction);
        }

        if (totalSimulationCount > LAG_APPROXIMATE)
        {
            valueFunction->updateValue(routingValueAtThisSimulation, assignmentValueAtThisSimulation, true);
        }
        else
        {
            valueFunction->updateValue(routingValueAtThisSimulation, assignmentValueAtThisSimulation, false);
        }
        totalSimulationCount++;
        routingValueAtThisSimulation.clear();
        assignmentValueAtThisSimulation.clear();
        for (auto iter = simulation.customers.begin(); iter != simulation.customers.end(); ++iter)
        {
            delete iter->second;
        }
        Generator::instanceGenenrator(false, &data, "");
        end = clock();
        cout << double(end - start) / CLOCKS_PER_SEC << endl;
    }
}
