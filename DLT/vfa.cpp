#include "vfa.h"

LookupTable::LookupTable()
{
    double initialValue = MAX_EDGE * double(CUSTOMER_NUMBER);
    double xTick = double(MAX_WORK_TIME) / double(LOOKUP_TABLE_INITIAL),
           yTick = double(MAX_WORK_TIME) / double(LOOKUP_TABLE_INITIAL);
    for (int xCount = 0; xCount < LOOKUP_TABLE_INITIAL; xCount++)
    {
        for (int yCount = 0; yCount < LOOKUP_TABLE_INITIAL; yCount++)
        {
            Entry newEntry;
            newEntry.x = xTick / 2.0 + double(xCount) * xTick;
            newEntry.y = yTick / 2.0 + double(yCount) * yTick;
            newEntry.xRange = xTick / 2.0;
            newEntry.yRange = yTick / 2.0;
            this->value[newEntry] = initialValue;
        }
    }
    for (int i = 0; i < int(MAX_WORK_TIME); i++)
    {
        for (int j = 0; j < int(MAX_WORK_TIME); j++)
        {
            this->tableValue[i][j] = initialValue;
        }
    }
}

double LookupTable::lookup(Aggregation postDecisionState)
{
    int lookX = floor(postDecisionState.currentTime), lookY = floor(postDecisionState.remainTime);
    return this->tableValue[lookX][lookY];
}

void LookupTable::partitionUpdate()
{
    map<Entry, double> entryTheta;
    int entryNum = this->value.size();
    double totalN = 0, totalTheta = 0;
    for (auto tableIter = this->value.begin(); tableIter != this->value.end(); ++tableIter)
    {
        totalN += this->tableInfo[tableIter->first].first;
        entryTheta[tableIter->first] = Util::standardDeviation(this->tableInfo[tableIter->first].second);
        totalTheta += entryTheta[tableIter->first];
    }
    //计算\hat{N}和\hat{theta}
    double averageN = totalN / entryNum, averageTheta = totalTheta / entryNum;
    auto tableIter = this->value.begin();
    for (int count = 0; count < entryNum; count++)
    {
        //计算N/\hat{N}和theta/\hat{theta}
        double factor1 = this->tableInfo[tableIter->first].first / averageN;
        double factor2 = entryTheta[tableIter->first] / averageTheta;
        if (factor1 * factor2 > PARTITION_THRESHOLD)
        {
            //若该entry 达到threshold，则对entry 进行再划分
            //cout << "partitioned entry: " << tableIter->first.x << " " << tableIter->first.y << endl;
            this->partition(tableIter);
            this->value.erase(tableIter++);
            for (auto iter = this->value.begin(); iter != this->value.end(); ++iter)
            {
                //cout << iter->first.x << " " << iter->first.y << endl;
            }
        }
        else
        {
            ++tableIter;
        }
    }
}

void LookupTable::partition(map<Entry, double>::iterator tableIter)
{
    //将当前entry 再划分为4个entry，并继承相关信息
    Entry partition1, partition2, partition3, partition4;
    partition1.x = tableIter->first.x + tableIter->first.xRange / 2.0;
    partition1.y = tableIter->first.y + tableIter->first.yRange / 2.0;
    partition1.xRange = tableIter->first.xRange / 2.0;
    partition1.yRange = tableIter->first.yRange / 2.0;
    partition2.x = tableIter->first.x + tableIter->first.xRange / 2.0;
    partition2.y = tableIter->first.y - tableIter->first.yRange / 2.0;
    partition2.xRange = tableIter->first.xRange / 2.0;
    partition2.yRange = tableIter->first.yRange / 2.0;
    partition3.x = tableIter->first.x - tableIter->first.xRange / 2.0;
    partition3.y = tableIter->first.y - tableIter->first.yRange / 2.0;
    partition3.xRange = tableIter->first.xRange / 2.0;
    partition3.yRange = tableIter->first.yRange / 2.0;
    partition4.x = tableIter->first.x - tableIter->first.xRange / 2.0;
    partition4.y = tableIter->first.y + tableIter->first.yRange / 2.0;
    partition4.xRange = tableIter->first.xRange / 2.0;
    partition4.yRange = tableIter->first.yRange / 2.0;
    this->value[partition1] = tableIter->second;
    this->value[partition2] = tableIter->second;
    this->value[partition3] = tableIter->second;
    this->value[partition4] = tableIter->second;
    this->tableInfo[partition1].first = this->tableInfo[tableIter->first].first / 4;
    this->tableInfo[partition2].first = this->tableInfo[tableIter->first].first / 4;
    this->tableInfo[partition3].first = this->tableInfo[tableIter->first].first / 4;
    this->tableInfo[partition4].first = this->tableInfo[tableIter->first].first / 4;
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
    this->remainTime = MAX_WORK_TIME - S.currentRoute->tail->arrivalTime;
}

Entry::Entry()
{
    x = 0.0;
    y = 0.0;
    xRange = 0.0;
    yRange = 0.0;
}

ValueFunction::ValueFunction()
{
    lookupTable = LookupTable();
}

double ValueFunction::getValue(Aggregation postDecisionState, double reward)
{
    return this->lookupTable.lookup(postDecisionState);
}

void ValueFunction::updateValue(vector<pair<Aggregation, double>> valueAtThisSimulation, bool startApproximate)
{
    double realValue = 0.0;
    for (auto decisionPoint = valueAtThisSimulation.rbegin(); decisionPoint != valueAtThisSimulation.rend(); ++decisionPoint)
    {
        realValue += decisionPoint->second;
        //cout << "point: " << decisionPoint->first.currentTime << " " << decisionPoint->first.remainTime << endl;
        for (auto tableIter = this->lookupTable.value.begin(); tableIter != this->lookupTable.value.end(); ++tableIter)
        {
            //对这次simulation 所查询过的entry 对应的value 进行更新
            if ((decisionPoint->first.currentTime >= tableIter->first.x - tableIter->first.xRange &&
                 decisionPoint->first.currentTime < tableIter->first.x + tableIter->first.xRange) &&
                (decisionPoint->first.remainTime >= tableIter->first.y - tableIter->first.yRange &&
                 decisionPoint->first.remainTime < tableIter->first.y + tableIter->first.yRange))
            {
                //cout << "entry: " << tableIter->first.x << " " << tableIter->first.y << endl;
                //记录该entry 的相关信息（被查找次数和更新的value）
                this->lookupTable.tableInfo[tableIter->first].first++;
                /*for (auto iter = this->lookupTable.tableInfo.begin(); iter != this->lookupTable.tableInfo.end(); ++iter)
                {
                    cout << iter->first.x << " " << iter->first.y << " " << iter->second.first << endl;
                }*/
                this->lookupTable.tableInfo[tableIter->first].second.push_back(decisionPoint->second);
                //更新value
                if (startApproximate)
                {
                    tableIter->second = (1 - STEP_SIZE) * tableIter->second + STEP_SIZE * realValue;
                }
                int minX = floor(tableIter->first.x - tableIter->first.xRange),
                    maxX = floor(tableIter->first.x + tableIter->first.xRange),
                    minY = floor(tableIter->first.y - tableIter->first.yRange),
                    maxY = floor(tableIter->first.y + tableIter->first.yRange);
                for (int i = minX; i < maxX; i++)
                {
                    for (int j = minY; j < maxY; j++)
                    {
                        this->lookupTable.tableValue[i][j] = tableIter->second;
                    }
                }
                break;
            }
        }
    }
    this->lookupTable.partitionUpdate();
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
