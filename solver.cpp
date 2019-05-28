#include "solver.h"

void Solver::solve()
{
    ValueFunction valueFunction;
    cout << "starting approximation!\n"
         << endl;
    //offline approximation
    AVI approximateValueIterate;
    //approximateValueIterate.approximation(&valueFunction);
    cout << "finished approximation!\n" << endl;

    //online solving
    int instanceNum = 0;
    vector<double> testResult;
    vector<double> rejectCost;
    while (instanceNum++ < MAX_TEST_INSTANCE)
    {
       char dayNum[] = {char(instanceNum / 1000000 + 48), char(instanceNum % 1000000 / 100000 + 48), char(instanceNum % 100000 / 10000 + 48),
                         char(instanceNum % 10000 / 1000 + 48), char(instanceNum % 1000 / 100 + 48),
                         char(instanceNum % 100 / 10 + 48), char(instanceNum % 10 + 48), '\0'};
        string fileName = "TestData/";
        fileName = fileName + dayNum + ".txt";
        MDP simulation = MDP(fileName);
        while (!simulation.sequenceData.empty() || !simulation.currentState.newCustomers.empty())
        {
            Action bestAction;
            double value;
            simulation.findBestAction(&bestAction, valueFunction, &value);
            //状态转移
            simulation.transition(bestAction);
        }
        testResult.push_back(simulation.solution.cost + simulation.cumRejectionReward);
        rejectCost.push_back(simulation.cumRejectionReward);
    }
    double resultSum = 0, rejectRate = 0.0;
    for (auto iter = testResult.begin(); iter != testResult.end(); ++iter)
    {
        resultSum += *iter;
    }
    for (auto iter = rejectCost.begin(); iter != rejectCost.end(); ++iter)
    {
        rejectRate += *iter / MAX_WORK_TIME;
    }
    cout << "Test Average Cost: " << resultSum / double(testResult.size()) << " " << rejectRate / double(rejectCost.size()) << endl;
    return;
}
