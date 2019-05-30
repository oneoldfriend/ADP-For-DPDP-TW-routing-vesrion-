#include "solver.h"
#include "generator.h"
#include <random>
#include <ctime>

void Solver::solve()
{
    //generate the test instance
    /*for (int instanceNum = 1; instanceNum <= MAX_TEST_INSTANCE; instanceNum++)
    {
        char dayNum[] = {char(instanceNum / 1000000 + 48), char(instanceNum % 1000000 / 100000 + 48), char(instanceNum % 100000 / 10000 + 48),
                         char(instanceNum % 10000 / 1000 + 48), char(instanceNum % 1000 / 100 + 48),
                         char(instanceNum % 100 / 10 + 48), char(instanceNum % 10 + 48), '\0'};
        string fileName = "TestData/";
        fileName = fileName + dayNum + ".txt";
        Generator::instanceGenenrator(fileName);
    }
    return;*/

    ValueFunction valueFunction;
    cout << "starting approximation!\n"
         << endl;
    //offline approximation
    AVI approximateValueIterate;
    if (!MYOPIC)
    {
        approximateValueIterate.approximation(&valueFunction);
    }
    cout << "finished approximation!\n"
         << endl;

    //online solving
    int instanceNum = 0;
    vector<double> testResult;
    while (instanceNum++ < MAX_TEST_INSTANCE)
    {
        char dayNum[] = {char(instanceNum / 1000000 + 48), char(instanceNum % 1000000 / 100000 + 48), char(instanceNum % 100000 / 10000 + 48),
                         char(instanceNum % 10000 / 1000 + 48), char(instanceNum % 1000 / 100 + 48),
                         char(instanceNum % 100 / 10 + 48), char(instanceNum % 10 + 48), '\0'};
        string fileName = "TestData/";
        fileName = fileName + dayNum + ".txt";
        MDP simulation = MDP(fileName);
        while (simulation.currentState.currentRoute != nullptr)
        {
            Action bestAction;
            double value;
            simulation.findBestAction(&bestAction, valueFunction, &value);
            //状态转移
            simulation.transition(bestAction);
        }
        simulation.solution.calcCost();
        testResult.push_back(simulation.solution.cost);
        cout << simulation.solution.cost << " " << simulation.solution.penalty << " " << simulation.solution.waitTime << " " << simulation.solution.travelTime << " " << simulation.currentState.notServicedCustomer.size() * MAX_WORK_TIME << endl;
    }
    double resultSum = 0;
    for (auto iter = testResult.begin(); iter != testResult.end(); ++iter)
    {
        resultSum += *iter;
    }
    cout << "Test Average Cost: " << resultSum / double(testResult.size()) << endl;
    return;
}
