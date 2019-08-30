#include "avi.h"
#include "generator.h"
#include <random>

void AVI::approximation(ValueFunction *valueFunction)
{
    //定义计数器，包括总模拟次数和每个instance的模拟次数
    bool initialization = true;
    int totalSimulationCount = 0, isSwitch = 0, switchCount = 0;
    list<pair<double, Customer *>> data;
    Generator::instanceGenenrator(false, &data, "");
    vector<pair<Eigen::VectorXd, double>> routingValueAtThisSimulation;
    vector<pair<Eigen::VectorXd, double>> assignmentValueAtThisSimulation;
    vector<pair<Eigen::VectorXd, double>> routingValueFromLastSwitch;
    vector<pair<Eigen::VectorXd, double>> assignmentValueFromLastSwitch;
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
            simulation.findBestAssignmentAction(&bestAction, *valueFunction, &assignmentReward, initialization);
            simulation.currentState.calcAttribute(bestAction, true);
            assignmentValueAtThisSimulation.push_back(make_pair(simulation.currentState.attributes, assignmentReward));
            assignmentValueFromLastSwitch.push_back(make_pair(simulation.currentState.attributes, assignmentReward));
            simulation.assignmentConfirmed(bestAction);
            simulation.findBestRoutingAction(&bestAction, *valueFunction, &routingReward, initialization);
            //记录这次sample path的信息
            simulation.executeAction(bestAction);
            simulation.currentState.calcAttribute(bestAction, false);
            simulation.undoAction(bestAction);
            routingValueAtThisSimulation.push_back(make_pair(simulation.currentState.attributes, routingReward));
            routingValueFromLastSwitch.push_back(make_pair(simulation.currentState.attributes, routingReward));
            //状态转移
            simulation.transition(bestAction);
        }

        simulation.solution.calcCost();

        if (isSwitch >= APPROXIMATION_SWITCH && switchCount < SWITCH_TIMES)
        {
            valueFunction->updateValue(routingValueAtThisSimulation, assignmentValueAtThisSimulation, &routingValueFromLastSwitch, &assignmentValueFromLastSwitch, false);
            valueFunction->updateValue(routingValueFromLastSwitch, assignmentValueFromLastSwitch, nullptr, nullptr, true);
            routingValueFromLastSwitch.clear();
            assignmentValueFromLastSwitch.clear();
            isSwitch = 0;
            switchCount++;
            if (initialization)
            {
                initialization = false;
            }
        }
        else
        {
            valueFunction->updateValue(routingValueAtThisSimulation, assignmentValueAtThisSimulation, &routingValueFromLastSwitch, &assignmentValueFromLastSwitch, false);
            isSwitch++;
        }
        totalSimulationCount++;
        routingValueAtThisSimulation.clear();
        assignmentValueAtThisSimulation.clear();
        if (SWITCH_TIMES == 0 && routingValueAtThisSimulation.size() != 0 && assignmentValueAtThisSimulation.size() != 0)
        {
            routingValueFromLastSwitch.clear();
            assignmentValueFromLastSwitch.clear();
        }
        for (auto iter = simulation.customers.begin(); iter != simulation.customers.end(); ++iter)
        {
            delete iter->second;
        }
        Generator::instanceGenenrator(false, &data, "");
        end = clock();
        cout << double(end - start) / CLOCKS_PER_SEC << endl;
    }
}
