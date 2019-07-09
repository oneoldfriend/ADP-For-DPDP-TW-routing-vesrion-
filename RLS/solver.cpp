#include "solver.h"
#include "generator.h"
#include <random>
#include <ctime>

void Solver::solve()
{
    //generate the test instance
    /*for (int instanceNum = 1; instanceNum <= MAX_TEST_INSTANCE; instanceNum++)
    {
        char dayNum[] = {char(CUSTOMER_NUMBER / 100 + 48), char(CUSTOMER_NUMBER % 100 / 10 + 48), char(CUSTOMER_NUMBER % 10 + 48), '-',
                         char(instanceNum / 1000000 + 48), char(instanceNum % 1000000 / 100000 + 48), char(instanceNum % 100000 / 10000 + 48),
                         char(instanceNum % 10000 / 1000 + 48), char(instanceNum % 1000 / 100 + 48),
                         char(instanceNum % 100 / 10 + 48), char(instanceNum % 10 + 48), '\0'};
        string fileName = "TestData/";
        fileName = fileName + dayNum + ".txt";
        Generator::instanceGenenrator(true, nullptr, fileName);
    }
    return;
    srand(time(NULL));*/

    ValueFunction valueFunction;
    //offline approximation
    AVI approximateValueIterate;
    if (!MYOPIC)
    {
        //cout << "starting approximation!\n"
        //     << endl;
        approximateValueIterate.approximation(&valueFunction);
        //cout << "finished approximation!\n"
        //     << endl;
    }

    //online solving
    int instanceNum = 0;
    vector<double> testResult;
    vector<double> rejection;
    while (instanceNum++ < MAX_TEST_INSTANCE)
    {
        char dayNum[] = {char(CUSTOMER_NUMBER / 100 + 48), char(CUSTOMER_NUMBER % 100 / 10 + 48), char(CUSTOMER_NUMBER % 10 + 48), '-',
                         char(instanceNum / 1000000 + 48), char(instanceNum % 1000000 / 100000 + 48), char(instanceNum % 100000 / 10000 + 48),
                         char(instanceNum % 10000 / 1000 + 48), char(instanceNum % 1000 / 100 + 48),
                         char(instanceNum % 100 / 10 + 48), char(instanceNum % 10 + 48), '\0'};
        string fileName = "TestData/";
        fileName = fileName + dayNum + ".txt";
        MDP simulation = MDP(false, fileName);
        while (simulation.currentState.currentRoute != nullptr)
        {
            Action bestAction;
            double routingReward = 0.0;
            simulation.findBestAssignmentAction(&bestAction, valueFunction);
            simulation.assignmentConfirmed(bestAction);
            simulation.findBestRoutingAction(&bestAction, valueFunction, &routingReward, false);
            //状态转移
            simulation.transition(bestAction);
        }
        simulation.solution.calcCost();
        testResult.push_back(simulation.solution.cost);
        rejection.push_back(simulation.cumOutsourcedCost);
        //cout << simulation.solution.cost << " " << simulation.solution.penalty << " " << simulation.solution.waitTime << " " << simulation.cumOutsourcedCost << " " << simulation.solution.cost + simulation.cumOutsourcedCost << endl;
    }
    double resultSum = 0, rejectionSum = 0;
    for (auto iter = testResult.begin(); iter != testResult.end(); ++iter)
    {
        resultSum += *iter;
    }
    for (auto iter = rejection.begin(); iter != rejection.end(); ++iter)
    {
        rejectionSum += *iter;
    }
    cout << "Test Average Cost: " << resultSum / double(testResult.size()) << " " << rejectionSum / double(rejection.size()) / MAX_WORK_TIME << " " << resultSum / double(testResult.size()) + rejectionSum / double(rejection.size()) << endl;
    return;
}
