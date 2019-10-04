#include "vfa.h"
#include <cmath>

LookupTable::LookupTable()
{
    averageN = 0;
    averageTheta = 0;
    double initialValue = -MAX_EDGE * double(CUSTOMER_NUMBER);
    double xTick, yTick;
    int entryCount = 0, xEntryNum, yEntryNum;
    if (DYNAMIC_LOOKUP_TABLE)
    {
        xEntryNum = 10 * MAX_VEHICLE;
        yEntryNum = 10 * MAX_VEHICLE;
        xTick = ceil(MAX_WORK_TIME / xEntryNum);
        yTick = ceil(MAX_VEHICLE * MAX_WORK_TIME / yEntryNum);
    }
    else
    {
        xTick = 5.0;
        yTick = 5.0;
        xEntryNum = (int)ceil(MAX_WORK_TIME / xTick),
        yEntryNum = (int)ceil(MAX_VEHICLE * MAX_WORK_TIME / yTick);
    }
    for (int i = 0; i < MAX_WORK_TIME; i++)
    {
        vector<int> secondD;
        for (int j = 0; j < MAX_VEHICLE * MAX_WORK_TIME; j++)
        {
            secondD.push_back(0);
        }
        this->entryIndex.push_back(secondD);
    }

    for (int xCount = 0; xCount < xEntryNum; xCount++)
    {
        for (int yCount = 0; yCount < yEntryNum; yCount++)
        {
            this->entryValue[entryCount] = initialValue;
            this->entryPosition[entryCount].first = floor(double(xCount) * xTick);
            this->entryPosition[entryCount].second = floor(double(yCount) * yTick);
            this->entryRange[entryCount].first = xTick;
            this->entryRange[entryCount].second = yTick;
            this->entryInfo[entryCount][0] = 0.0;
            this->entryInfo[entryCount][1] = 0.0;
            this->entryInfo[entryCount][2] = 0.0;
            for (int i = (int)this->entryPosition[entryCount].first;
                 i < (int)floor(this->entryPosition[entryCount].first + this->entryRange[entryCount].first) &&
                 i < (int)MAX_WORK_TIME;
                 i++)
            {
                for (int j = (int)this->entryPosition[entryCount].second;
                     j < (int)floor(this->entryPosition[entryCount].second + this->entryRange[entryCount].second) &&
                     j < (int)MAX_VEHICLE * MAX_WORK_TIME;
                     j++)
                {
                    this->entryIndex[i][j] = entryCount;
                }
            }
            entryCount++;
        }
    }
}

double LookupTable::lookup(Aggregation postDecisionState)
{
    int lookX = (int)floor(postDecisionState.currentTime), lookY = (int)floor(postDecisionState.remainTime);
    return this->entryValue[this->entryIndex[lookX][lookY]];
}

void LookupTable::partitionCheck()
{
    double entrySize = this->entryValue.size();
    double newAverageN = this->averageN, newAverageTheta = this->averageTheta;
    for (int entryCount = 0; entryCount < entrySize; entryCount++)
    {
        double factor1 = this->entryInfo[entryCount][0] / this->averageN;
        double entryTheta = sqrt(this->entryInfo[entryCount][1] / max(this->entryInfo[entryCount][0] - 1.0, 1.0));
        double factor2 = entryTheta / this->averageTheta;
        if (factor1 * factor2 > (double)PARTITION_THRESHOLD)
        {
            this->partition(entryCount);
            double i = 3.0;
            while (i-- > 0.0)
            {
                newAverageN += (this->entryInfo[entryCount][0] - newAverageN) / (this->entryValue.size() - i);
                newAverageTheta += (entryTheta - newAverageTheta) / (this->entryValue.size() - i);
            }
        }
    }
    this->averageN = newAverageN;
    this->averageTheta = newAverageTheta;
}

void LookupTable::partition(int entryNum)
{
    //将当前entry 再划分为4个entry，并继承相关信息
    int newEntryIndex = this->entryValue.size(), entrySize = this->entryValue.size();
    this->entryRange[entryNum].first = this->entryRange[entryNum].first / 2.0;
    this->entryRange[entryNum].second = this->entryRange[entryNum].second / 2.0;
    this->entryInfo[entryNum][0] = this->entryInfo[entryNum][0] / 4.0;
    this->entryInfo[entryNum][1] = this->entryInfo[entryNum][1] / 16.0;
    this->entryPosition[newEntryIndex].first = this->entryPosition[entryNum].first;
    this->entryPosition[newEntryIndex].second = floor(this->entryPosition[entryNum].second + this->entryRange[entryNum].second);
    this->entryPosition[newEntryIndex + 1].first = floor(this->entryPosition[entryNum].first + this->entryRange[entryNum].first);
    this->entryPosition[newEntryIndex + 1].second = this->entryPosition[entryNum].second;
    this->entryPosition[newEntryIndex + 2].first = floor(this->entryPosition[entryNum].first + this->entryRange[entryNum].first);
    this->entryPosition[newEntryIndex + 2].second = floor(this->entryPosition[entryNum].second + this->entryRange[entryNum].second);
    while (newEntryIndex < entrySize + 3)
    {
        this->entryValue[newEntryIndex] = this->entryValue[entryNum];
        this->entryRange[newEntryIndex].first = this->entryRange[entryNum].first;
        this->entryRange[newEntryIndex].second = this->entryRange[entryNum].second;
        this->entryInfo[newEntryIndex][0] = this->entryInfo[entryNum][0];
        this->entryInfo[newEntryIndex][1] = this->entryInfo[entryNum][1];
        this->entryInfo[newEntryIndex][2] = this->entryInfo[entryNum][2];
        for (int i = (int)this->entryPosition[newEntryIndex].first;
             i < (int)this->entryPosition[newEntryIndex].first + this->entryRange[newEntryIndex].first &&
             i < (int)MAX_WORK_TIME;
             i++)
        {
            for (int j = (int)this->entryPosition[newEntryIndex].second;
                 j < (int)this->entryPosition[newEntryIndex].second + this->entryRange[newEntryIndex].second &&
                 j < (int)MAX_VEHICLE * MAX_WORK_TIME;
                 j++)
            {
                this->entryIndex[i][j] = newEntryIndex;
            }
        }
        newEntryIndex++;
    }
}

void LookupTable::updateTable(Aggregation postDecisionState, double value)
{
    int lookX = (int)floor(postDecisionState.currentTime), lookY = (int)floor(postDecisionState.remainTime);
    int entryNum = this->entryIndex[lookX][lookY];
    this->averageN -= this->entryInfo[entryNum][0] / this->entryValue.size();
    this->averageTheta -= sqrt(this->entryInfo[entryNum][1] / max(this->entryInfo[entryNum][0] - 1.0, 1.0)) / this->entryValue.size();
    this->entryInfo[entryNum][0]++;
    this->entryInfo[entryNum][1] += (this->entryInfo[entryNum][0] - 1.0) / this->entryInfo[entryNum][0] * pow(value - this->entryInfo[entryNum][2], 2);
    this->entryInfo[entryNum][2] += (value - this->entryInfo[entryNum][2]) / this->entryInfo[entryNum][0];
    this->averageN += this->entryInfo[entryNum][0] / this->entryValue.size();
    this->averageTheta += sqrt(this->entryInfo[entryNum][1] / max(this->entryInfo[entryNum][0] - 1.0, 1.0)) / this->entryValue.size();
    this->entryValue[entryNum] = (1.0 - STEP_SIZE) * this->entryValue[entryNum] + STEP_SIZE * value;
}

Aggregation::Aggregation()
{
    currentTime = 0;
    remainTime = 0;
}

void Aggregation::aggregate(State S, Action a)
{
    //对执行动作后的解进行相关的信息提取
    this->currentTime = S.currentTime;
    for (auto iter = S.pointSolution->routes.begin(); iter != S.pointSolution->routes.end(); ++iter)
    {
        this->remainTime += (double)MAX_WORK_TIME - iter->tail->arrivalTime;
    }
}

ValueFunction::ValueFunction()
{
    assignmentLookupTable = LookupTable();
    routingLookupTable = LookupTable();
}

double ValueFunction::getValue(State S, Action a, bool assignment, bool myopic)
{
    Aggregation postDecisionState;
    postDecisionState.aggregate(S, a);
    if (assignment)
    {
        if (ASSIGNMENT_MYOPIC || myopic)
        {
            return 0;
        }
        else
        {
            return this->assignmentLookupTable.lookup(postDecisionState);
        }
    }
    else
    {
        if (ROUTING_MYOPIC || myopic)
        {
            return 0;
        }
        else
        {
            return this->routingLookupTable.lookup(postDecisionState);
        }
    }
}

void ValueFunction::updateValue(vector<pair<Aggregation, double>> assignmentValueAtThisSimulation, vector<pair<Aggregation, double>> routingValueAtThisSimulation, bool startApproximate)
{
    double lastValue = 0.0;
    for (int i = 0; i < (int)assignmentValueAtThisSimulation.size(); i++)
    {
        routingValueAtThisSimulation[i].second += assignmentValueAtThisSimulation[i].second;
    }
    for (auto iter = routingValueAtThisSimulation.rbegin(); iter != routingValueAtThisSimulation.rend(); ++iter)
    {
        iter->second += lastValue;
        lastValue = double(LAMBDA) * iter->second;
    }
    for (int i = 0; i < (int)assignmentValueAtThisSimulation.size(); i++)
    {
        assignmentValueAtThisSimulation[i].second = routingValueAtThisSimulation[i].second;
    }

    for (int i = 0; i < (int)routingValueAtThisSimulation.size(); i++)
    {
        this->assignmentLookupTable.updateTable(assignmentValueAtThisSimulation[i].first, assignmentValueAtThisSimulation[i].second);
        this->routingLookupTable.updateTable(routingValueAtThisSimulation[i].first, routingValueAtThisSimulation[i].second);
    }
    if (DYNAMIC_LOOKUP_TABLE)
    {
        if (startApproximate)
        {
            this->assignmentLookupTable.partitionCheck();
            this->routingLookupTable.partitionCheck();
        }
    }
}