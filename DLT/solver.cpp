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
        string fileName = "/home/linfei/ADP-For-DPDP-TW-routing-vesrion-/RLS/TestData/TestData/";
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
        //cout << "starting approximation!\n" << endl;
        approximateValueIterate.approximation(&valueFunction);
        //cout << "finished approximation!\n" << endl;
    }

    //online solving
    int instanceNum = 0;
    vector<double> testResult;
    vector<double> rejection;
    vector<double> latenessForAll;
    vector<double> latenessForReminder;
    /*for (int i = 0; i < (int)MAX_WORK_TIME; i++)
    {
        for (int j = 0; j < (int)MAX_VEHICLE * MAX_WORK_TIME; j++)
        {
            int entryNum = valueFunction.lookupTable.entryIndex[i][j];
            cout << i << " " << j << " " << entryNum << " " << valueFunction.lookupTable.entryValue[entryNum] << endl;
        }
    }*/
    while (instanceNum++ < MAX_TEST_INSTANCE)
    {
        double duration;
        clock_t start, end;
        start = clock();
        char dayNum[] = {char(CUSTOMER_NUMBER / 100 + 48), char(CUSTOMER_NUMBER % 100 / 10 + 48), char(CUSTOMER_NUMBER % 10 + 48), '-',
                         char(instanceNum / 1000000 + 48), char(instanceNum % 1000000 / 100000 + 48), char(instanceNum % 100000 / 10000 + 48),
                         char(instanceNum % 10000 / 1000 + 48), char(instanceNum % 1000 / 100 + 48),
                         char(instanceNum % 100 / 10 + 48), char(instanceNum % 10 + 48), '\0'};
        string fileName = "/home/linfei/ADP-For-DPDP-TW-routing-vesrion-/RLS/TestData/";
        fileName = fileName + dayNum + ".txt";
        MDP simulation = MDP(false, fileName);
        while (simulation.currentState.currentRoute != nullptr)
        {
            Action bestAction;
            double routingReward = 0.0;
            simulation.findBestAssignmentAction(&bestAction, &valueFunction);
            simulation.assignmentConfirmed(bestAction);
            simulation.findBestRoutingAction(&bestAction, &valueFunction, &routingReward, false);
            //状态转移
            simulation.transition(bestAction);
        }
        simulation.solution.calcCost();
        testResult.push_back(simulation.solution.cost);
        rejection.push_back(simulation.cumOutsourcedCost);
        double allLateness = 0.0, reminderLateness = 0.0;
        for (auto iter = simulation.solution.routes.begin(); iter != simulation.solution.routes.end(); ++iter)
        {
            PointOrder p = iter->head->next;
            while (p != iter->tail)
            {
                if (p->customer->startTime != 0 && !p->isOrigin)
                {
                    allLateness += max(0.0, p->arrivalTime - p->customer->endTime);
                }
                if (p->customer->priority == 2 && !p->isOrigin)
                {
                    reminderLateness += max(0.0, p->arrivalTime - p->customer->endTime);
                }
                p = p->next;
            }
        }
        latenessForReminder.push_back(reminderLateness);
        latenessForAll.push_back(allLateness);
        end = clock();
        duration = (double)(end - start) / CLOCKS_PER_SEC;
        //cout << "last " << duration << "s  " << endl;
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
    double resultSum = 0, rejectionSum = 0, allLatenessSum = 0.0, reminderLatenessSum = 0.0;
    for (int index = 0; index < MAX_TEST_INSTANCE; index++)
    {
        resultSum += testResult[index];
        rejectionSum += rejection[index];
        allLatenessSum += latenessForAll[index];
        reminderLatenessSum += latenessForReminder[index];
    }
    cout << "Test Average Cost: " << resultSum / double(MAX_TEST_INSTANCE) << " " << resultSum / double(MAX_TEST_INSTANCE) + rejectionSum / double(MAX_TEST_INSTANCE) << " " << allLatenessSum / (double)MAX_TEST_INSTANCE << " " << reminderLatenessSum / (double)MAX_TEST_INSTANCE << endl;
    return;
}
