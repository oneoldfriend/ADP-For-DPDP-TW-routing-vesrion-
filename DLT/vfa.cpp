#include "vfa.h"
#include <cmath>

LookupTable::LookupTable()
{
    double initialValue = -MAX_EDGE * double(CUSTOMER_NUMBER);
    double xTick = 5.0, yTick = 5.0;
    int entryCount = 0,
        xEntryNum = 10, //(int)ceil(MAX_WORK_TIME / xTick),
        yEntryNum = 10; //(int)ceil(MAX_VEHICLE * MAX_WORK_TIME / yTick);

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

void LookupTable::partitionUpdate()
{
    map<int, double> entryTheta;
    int entrySize = this->entryValue.size();
    double totalN = 0, totalTheta = 0;
    for (int entryCount = 0; entryCount < entrySize; entryCount++)
    {
        totalN += this->entryInfo[entryCount].first;
        double vectorCopy[this->entryInfo[entryCount].second.size()];
        for (int i = 0; i < this->entryInfo[entryCount].second.size(); i++)
        {
            vectorCopy[i] = this->entryInfo[entryCount].second[i];
        }
        entryTheta[entryCount] = Util::standardDeviation(vectorCopy, this->entryInfo[entryCount].second.size());
        totalTheta += entryTheta[entryCount];
    }
    //计算\hat{N}和\hat{theta}
    double averageN = totalN / entrySize, averageTheta = totalTheta / entrySize;
    for (int entryCount = 0; entryCount < entrySize; entryCount++)
    {
        //计算N/\hat{N}和theta/\hat{theta}
        double factor1 = this->entryInfo[entryCount].first / averageN;
        double factor2 = entryTheta[entryCount] / averageTheta;
        if (factor1 * factor2 > PARTITION_THRESHOLD)
        {
            //cout << factor1 << " " << factor2 << " " << PARTITION_THRESHOLD << endl;
            //若该entry 达到threshold，则对entry 进行再划分
            //cout << "partitioned entry: " << this->entryPosition[entryCount].first << " " << this->entryPosition[entryCount].second << endl;
            this->partition(entryCount);
        }
    }
}

void LookupTable::partition(int entryNum)
{
    //将当前entry 再划分为4个entry，并继承相关信息
    int newEntryIndex = this->entryValue.size(), entrySize = this->entryValue.size();
    this->entryRange[entryNum].first = this->entryRange[entryNum].first / 2.0;
    this->entryRange[entryNum].second = this->entryRange[entryNum].second / 2.0;
    this->entryInfo[entryNum].first = this->entryInfo[entryNum].first / 4;
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
        this->entryInfo[newEntryIndex].first = this->entryInfo[entryNum].first;
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
        this->lookupTable.entryInfo[entryNum].first++;
        this->lookupTable.entryInfo[entryNum].second.push_back(decisionPoint->second);
        errorThisSimulation += abs(this->lookupTable.entryValue[entryNum] - decisionPoint->second);
        this->lookupTable.entryValue[entryNum] = (1 - STEP_SIZE) * this->lookupTable.entryValue[entryNum] + STEP_SIZE * decisionPoint->second;
    }
    cout << errorThisSimulation << endl;
    if (DYNAMIC_LOOKUP_TABLE)
    {
        this->lookupTable.partitionUpdate();
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
