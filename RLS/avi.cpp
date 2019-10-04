#include "avi.h"
#include "generator.h"
#include <random>

void AVI::approximation(ValueFunction *valueFunction)
{
    //定义计数器，包括总模拟次数和每个instance的模拟次数
    int totalSimulationCount = 0;
    list<pair<double, Customer *>> data;
    Generator::instanceGenenrator(false, &data, "");
    vector<pair<Eigen::VectorXd, double>> routingValueAtThisSimulation;
    vector<pair<Eigen::VectorXd, double>> assignmentValueAtThisSimulation;
    vector<pair<Eigen::VectorXd, pair<Eigen::VectorXd, double>>> routingReplayBuffer;
    vector<pair<Eigen::VectorXd, pair<Eigen::VectorXd, double>>> assignmentReplayBuffer;
    while (totalSimulationCount < MAX_SIMULATION)
    {
        clock_t start, end;
        start = clock();

        //初始化马尔科夫决策过程
        MDP simulation = MDP(true, "", &data);
        //开始mdp模拟
        Eigen::VectorXd lastRoutingState;
        Eigen::VectorXd lastAssignmentState;
        bool justStart = true;
        while (simulation.currentState.currentRoute != nullptr)
        {
            Action bestAction;
            double routingReward = 0.0, assignmentReward = 0.0;
            simulation.findBestAssignmentAction(&bestAction, *valueFunction, &assignmentReward, false);
            simulation.currentState.calcAttribute(bestAction, true);
            pair<Eigen::VectorXd, double> assignmentObservation = make_pair(simulation.currentState.attributes, assignmentReward);
            assignmentValueAtThisSimulation.push_back(assignmentObservation);
            simulation.assignmentConfirmed(bestAction);
            simulation.findBestRoutingAction(&bestAction, *valueFunction, &routingReward, false);
            //记录这次sample path的信息
            simulation.executeAction(bestAction);
            simulation.currentState.calcAttribute(bestAction, false);
            simulation.undoAction(bestAction);
            pair<Eigen::VectorXd, double> routingObservation = make_pair(simulation.currentState.attributes, routingReward);
            routingValueAtThisSimulation.push_back(routingObservation);
            //状态转移
            simulation.transition(bestAction);
            if (justStart)
            {
                justStart = false;
            }
            else
            {
                assignmentReplayBuffer.push_back(make_pair(lastAssignmentState, assignmentObservation));
                routingReplayBuffer.push_back(make_pair(lastRoutingState, routingObservation));
            }
            lastAssignmentState = assignmentObservation.first;
            lastRoutingState = routingObservation.first;
        }
        assignmentReplayBuffer.pop_back();
        routingReplayBuffer.pop_back();

        if (totalSimulationCount > LAG_APPROXIMATE)
        {
            valueFunction->updateValue(routingValueAtThisSimulation, assignmentValueAtThisSimulation, true);
        }
        else
        {
            valueFunction->updateValue(routingValueAtThisSimulation, assignmentValueAtThisSimulation, false);
        }
        totalSimulationCount++;
        cout << totalSimulationCount << " ";
        if (totalSimulationCount % REVIEW_MAX == 0)
        {
            //valueFunction->reObservationUpdate(routingReplayBuffer, assignmentReplayBuffer);
            routingReplayBuffer.clear();
            assignmentReplayBuffer.clear();
        }
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
