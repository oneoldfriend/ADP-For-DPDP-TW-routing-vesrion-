#include "solver.h"
#include "generator.h"
#include <algorithm>
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
    vector<double> consolidation;
    vector<double> visitForNothing;
    vector<double> travelCost;
    vector<double> penaltyCost;
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
        double consolidationCount = 0.0, visitForNothingCount = 0.0;
        for (auto iter = simulation.solution.routes.begin(); iter != simulation.solution.routes.end(); ++iter)
        {
            PointOrder p = iter->head->next;
            while (p != iter->tail)
            {
                if (p->isOrigin && p->next->isOrigin)
                {
                    consolidationCount += 1;
                }
                if (p->customer->priority == 0 && p->isOrigin)
                {
                    visitForNothingCount += 1; //max(0.0, p->arrivalTime - p->customer->endTime);
                }
                p = p->next;
            }
        }
        consolidation.push_back(consolidationCount);
        visitForNothing.push_back(visitForNothingCount);
        travelCost.push_back(simulation.solution.travelTime);
        penaltyCost.push_back(simulation.solution.penalty);
        //cout << simulation.solution.cost << " " << simulation.solution.penalty << " " << simulation.solution.waitTime << " " << simulation.cumOutsourcedCost << " " << simulation.solution.cost + simulation.cumOutsourcedCost << endl;
        /*ofstream outFile("solution.txt", ios::out);
        for (auto iter = simulation.solution.routes.begin(); iter != simulation.solution.routes.end(); ++iter)
        {
            PointOrder p = iter->head;
            while (p != nullptr)
            {
                outFile << p->position.x << ",";
                outFile << p->position.y << endl;
                p = p->next;
            }
        }
        outFile.close();*/
    }
    double resultSum = 0, rejectionSum = 0, consolidationSum = 0.0, visitForNothingSum = 0.0, travelCostSum = 0.0, penaltyCostSum = 0.0;
    for (int index = 0; index < MAX_TEST_INSTANCE; index++)
    {
        resultSum += testResult[index];
        rejectionSum += rejection[index];
        consolidationSum += consolidation[index];
        visitForNothingSum += visitForNothing[index];
        travelCostSum += travelCost[index];
        penaltyCostSum += penaltyCost[index];
    }
    cout << "Test Average Cost: " << resultSum / double(MAX_TEST_INSTANCE) << " " << travelCostSum / double(MAX_TEST_INSTANCE) << " " << penaltyCostSum / double(MAX_TEST_INSTANCE) << " " << resultSum / double(MAX_TEST_INSTANCE) + rejectionSum / double(MAX_TEST_INSTANCE) << " " << consolidationSum / (double)MAX_TEST_INSTANCE << " " << visitForNothingSum / (double)MAX_TEST_INSTANCE << endl;
    return;
}
