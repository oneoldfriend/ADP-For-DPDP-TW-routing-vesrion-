#include "vfa.h"
#include <cmath>

LookupTable::LookupTable()
{
    averageN = 0;
    averageTheta = 0;
    double initialValue = -MAX_EDGE * double(CUSTOMER_NUMBER);
    double xTick = 5.0, yTick = 5.0;
    int entryCount = 0,
        xEntryNum = (int)ceil(MAX_WORK_TIME / xTick),
        yEntryNum = (int)ceil(MAX_VEHICLE * MAX_WORK_TIME / yTick);

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
    cout << entrySize << endl;
    double newAverageN = this->averageN, newAverageTheta = this->averageTheta;
    for (int entryCount = 0; entryCount < entrySize; entryCount++)
    {
        double factor1 = this->entryInfo[entryCount][0] / this->averageN;
        double entryTheta = sqrt(this->entryInfo[entryCount][1] / max(this->entryInfo[entryCount][0] - 1.0, 1.0));
        double factor2 = entryTheta / this->averageTheta;
        if (factor1 * factor2 > (double)PARTITION_THRESHOLD)
        {
            //cout << factor1 << " " << factor2 << " " << PARTITION_THRESHOLD << endl;
            //若该entry 达到threshold，则对entry 进行再划分
            //cout << "partitioned entry: " << this->entryPosition[entryCount].first << " " << this->entryPosition[entryCount].second << endl;
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
             i < (int)this->entryPosition[newEntryIndex].first + this->entryRange[newEntryIndex].first;
             i++)
        {
            for (int j = (int)this->entryPosition[newEntryIndex].second;
                 j < (int)this->entryPosition[newEntryIndex].second + this->entryRange[newEntryIndex].second;
                 j++)
            {
                this->entryIndex[i][j] = newEntryIndex;
            }
        }
        newEntryIndex++;
    }
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
    lookupTable = LookupTable();
}

double ValueFunction::getValue(State S, Action a, bool approx)
{
    if (MYOPIC)
    {
        return 0.0;
    }
    else
    {
        Aggregation postDecisionState;
        postDecisionState.aggregate(S, a);
        return this->lookupTable.lookup(postDecisionState);
    }
}

void ValueFunction::updateValue(vector<pair<Aggregation, double>> valueAtThisSimulation, bool startApproximate)
{
    double lastValue = 0;
    double errorThisSimulation = 0.0;
    for (auto iter = valueAtThisSimulation.rbegin(); iter != valueAtThisSimulation.rend(); ++iter)
    {
        iter->second += lastValue;
        lastValue = double(LAMBDA) * iter->second;
    }
    for (auto decisionPoint = valueAtThisSimulation.begin(); decisionPoint != valueAtThisSimulation.end(); ++decisionPoint)
    {
        int lookX = (int)floor(decisionPoint->first.currentTime), lookY = (int)floor(decisionPoint->first.remainTime);
        int entryNum = this->lookupTable.entryIndex[lookX][lookY];
        this->lookupTable.averageN -= this->lookupTable.entryInfo[entryNum][0] / this->lookupTable.entryValue.size();
        this->lookupTable.averageTheta -= sqrt(this->lookupTable.entryInfo[entryNum][1] / max(this->lookupTable.entryInfo[entryNum][0] - 1.0, 1.0)) / this->lookupTable.entryValue.size();
        this->lookupTable.entryInfo[entryNum][0]++;
        this->lookupTable.entryInfo[entryNum][1] += (this->lookupTable.entryInfo[entryNum][0] - 1.0) / this->lookupTable.entryInfo[entryNum][0] * pow(decisionPoint->second - this->lookupTable.entryInfo[entryNum][2], 2);
        this->lookupTable.entryInfo[entryNum][2] += (decisionPoint->second - this->lookupTable.entryInfo[entryNum][2]) / this->lookupTable.entryInfo[entryNum][0];
        this->lookupTable.averageN += this->lookupTable.entryInfo[entryNum][0] / this->lookupTable.entryValue.size();
        this->lookupTable.averageTheta += sqrt(this->lookupTable.entryInfo[entryNum][1] / max(this->lookupTable.entryInfo[entryNum][0] - 1.0, 1.0)) / this->lookupTable.entryValue.size();
        errorThisSimulation += abs(this->lookupTable.entryValue[entryNum] - decisionPoint->second);
        this->lookupTable.entryValue[entryNum] = (1.0 - STEP_SIZE) * this->lookupTable.entryValue[entryNum] + STEP_SIZE * decisionPoint->second;
    }
    cout << errorThisSimulation / valueAtThisSimulation.size() << endl;
    if (DYNAMIC_LOOKUP_TABLE)
    {
        if (startApproximate)
        {
            this->lookupTable.partitionCheck();
        }
    }
}

/*ValueFunction::ValueFunction()
{
    lookupTable = LookupTable();
    lambda = 1;
    for (int i = 0; i < ATTRIBUTES_NUMBER; i++)
    {
        attributesWeight[i] = 1.0;
    }
    this->matrixBeta = Eigen::Matrix4d::Identity();
}

double ValueFunction::getValue(State S, Action a)
{
    Solution tempSolution = Solution();
    tempSolution.solutionCopy(S.pointSolution);
    tempSolution.calcAttribute();
    double value = this->attributesWeight.transpose() * tempSolution.attribute;
    tempSolution.solutionDelete();
    if (MYOPIC)
    {
        return 0;
    }
    return value;
}

void ValueFunction::updateValue(vector<pair<Eigen::Vector4d, double>> valueAtThisSimulation, bool startApproximate)
{
    double realValue = 0;
    double errorThisSimulation = 0.0;
    for (auto iter = valueAtThisSimulation.rbegin(); iter != valueAtThisSimulation.rend(); ++iter)
    {
        realValue += iter->second;
        //cout << "real value: " << realValue << endl;
        //cout << "estimated value(before update):" << this->attributesWeight.transpose() * iter->first << endl;
        double gammaN = this->lambda + iter->first.transpose() * this->matrixBeta * iter->first,
               error = abs(this->attributesWeight.transpose() * iter->first - realValue);
        this->matrixBeta = 1.0 / this->lambda * (this->matrixBeta - 1.0 / gammaN * (this->matrixBeta * iter->first * iter->first.transpose() * this->matrixBeta));
        this->attributesWeight = this->attributesWeight - 1 / gammaN * this->matrixBeta.inverse().inverse() * iter->first * error;
        //cout << "estimated value(after update):" << this->attributesWeight.transpose() * iter->first << endl;
        errorThisSimulation += error;
    }
    cout << errorThisSimulation << endl;
}*/
