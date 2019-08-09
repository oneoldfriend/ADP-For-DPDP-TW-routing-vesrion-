#include "vfa.h"
#include <iomanip>

LookupTable::LookupTable()
{
    double initialValue = -MAX_EDGE * double(CUSTOMER_NUMBER);
    double xTick = double(MAX_WORK_TIME) / double(LOOKUP_TABLE_INITIAL),
           yTick = double(MAX_WORK_TIME) * double(MAX_VEHICLE) / double(LOOKUP_TABLE_INITIAL);
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
    for (auto iter = this->value.begin(); iter != this->value.end(); ++iter)
    {
        //cout << iter->first.x << " " << iter->first.y << " " << iter->first.xRange << " " << iter->first.yRange << endl;
    }
}

double LookupTable::lookup(Aggregation postDecisionState)
{
    //找到aggregation 在lookup table 中对应的entry，并返回其value
    for (auto iter = this->value.begin(); iter != this->value.end(); ++iter)
    {
        if ((postDecisionState.currentTime >= iter->first.x - iter->first.xRange &&
             postDecisionState.currentTime < iter->first.x + iter->first.xRange) &&
            (postDecisionState.remainTime >= iter->first.y - iter->first.yRange &&
             postDecisionState.remainTime < iter->first.y + iter->first.yRange))
        {
            this->tableInfo[iter->first].first = this->tableInfo[iter->first].first + 1;
            return iter->second;
        }
    }
    return 0.0;
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
    for (auto iter = S.pointSolution->routes.begin(); iter != S.pointSolution->routes.end(); ++iter)
    {
        this->remainTime += (double)MAX_WORK_TIME - iter->tail->arrivalTime;
    }
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
    lambda = 1;
    this->postdecisionAttributesWeight = Eigen::VectorXd(ATTRIBUTES_NUMBER);
    this->predecisionAttributesWeight = Eigen::VectorXd(ATTRIBUTES_NUMBER);
    this->postdecisionMatrixBeta = Eigen::MatrixXd(ATTRIBUTES_NUMBER, ATTRIBUTES_NUMBER);
    this->predecisionMatrixBeta = Eigen::MatrixXd(ATTRIBUTES_NUMBER, ATTRIBUTES_NUMBER);
    for (int i = 0; i < ATTRIBUTES_NUMBER; i++)
    {
        this->postdecisionAttributesWeight(i) = 1.0;
        this->predecisionAttributesWeight(i) = 1.0;
        for (int j = 0; j < ATTRIBUTES_NUMBER; j++)
        {
            if (i == j)
            {
                this->postdecisionMatrixBeta(i, j) = MAX_EDGE * CUSTOMER_NUMBER / MAX_VEHICLE;
                this->predecisionMatrixBeta(i, j) = MAX_EDGE * CUSTOMER_NUMBER / MAX_VEHICLE;
            }
            else
            {
                this->postdecisionMatrixBeta(i, j) = 0;
                this->predecisionMatrixBeta(i, j) = 0;
            }
        }
    }
}

/*double ValueFunction::getValue(Aggregation postDecisionState, double reward)
{
    return this->lookupTable.lookup(postDecisionState);
}*/

double ValueFunction::getValue(State S, Action a, bool predecision)
{
    S.calcAttribute(a, false);
    if (MYOPIC)
    {
        return 0;
    }
    else
    {
        if (false)
        {
            return this->predecisionAttributesWeight.transpose() * S.attributes;
        }
        else
        {
            return this->postdecisionAttributesWeight.transpose() * S.attributes;
        }
    }
}

/*void ValueFunction::updateValue(vector<pair<Aggregation, double> > valueAtThisSimulation, bool startApproximate)
{
    for (auto decisionPoint = valueAtThisSimulation.begin(); decisionPoint != valueAtThisSimulation.end(); ++decisionPoint)
    {
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
                for (auto iter = this->lookupTable.tableInfo.begin(); iter != this->lookupTable.tableInfo.end(); ++iter)
                {
                    //cout << iter->first.x << " " << iter->first.y << " " << iter->second.first << endl;
                }
                this->lookupTable.tableInfo[tableIter->first].second.push_back(decisionPoint->second);
                //更新value
                if (startApproximate)
                {
                    tableIter->second = (1 - STEP_SIZE) * tableIter->second + STEP_SIZE * decisionPoint->second;
                }
                break;
            }
        }
    }
    this->lookupTable.partitionUpdate();
}*/

void ValueFunction::updateValue(vector<pair<Eigen::VectorXd, double> > postdecisionValueAtThisSimulation, vector<pair<Eigen::VectorXd, double> > predecisionValueAtThisSimulation, bool startApproximate)
{
    //this->matrixBeta = MAX_EDGE * CUSTOMER_NUMBER / MAX_VEHICLE * Eigen::Matrix4d::Identity();
    //this->matrixBeta = Eigen::Matrix4d::Identity();
    double lastValue = 0;
    double predecisionEstimatorErrorThisSimulation = 0.0, postdecisionEstimatorErrorThisSimulation = 0.0;
    vector<double> rewardStored;
    for (auto iter = postdecisionValueAtThisSimulation.rbegin(); iter != postdecisionValueAtThisSimulation.rend(); ++iter)
    {
        //double reward = iter->second;
        rewardStored.push_back(iter->second);
        iter->second += lastValue;
        lastValue = double(NOISE_DEDUCTION) * iter->second;
    }
    for (int i = 0; i < (int)predecisionValueAtThisSimulation.size(); i++)
    {
        predecisionValueAtThisSimulation[i].second = postdecisionValueAtThisSimulation[i].second;
    }

    //update for pre decision value estimator
    for (int i = 0; i < (int)predecisionValueAtThisSimulation.size(); i++)
    {
        double gammaN = 1.0 + predecisionValueAtThisSimulation[i].first.transpose() * this->predecisionMatrixBeta * predecisionValueAtThisSimulation[i].first,
               error = ALPHA * (this->predecisionAttributesWeight.transpose() * predecisionValueAtThisSimulation[i].first - predecisionValueAtThisSimulation[i].second) + (1 - ALPHA) * (this->predecisionAttributesWeight.transpose() * predecisionValueAtThisSimulation[i].first - (this->postdecisionAttributesWeight.transpose() * postdecisionValueAtThisSimulation[i].first + rewardStored[i]));
        this->predecisionAttributesWeight = this->predecisionAttributesWeight - 1 / gammaN * this->predecisionMatrixBeta * predecisionValueAtThisSimulation[i].first * error;
        //cout << 1 / gammaN * this->matrixBeta << endl;
        this->predecisionMatrixBeta = this->predecisionMatrixBeta - 1.0 / gammaN * (this->predecisionMatrixBeta * predecisionValueAtThisSimulation[i].first * predecisionValueAtThisSimulation[i].first.transpose() * this->predecisionMatrixBeta);
        predecisionEstimatorErrorThisSimulation += abs(error);
    }

    //update for post decision value estimator
    for (int i = 0; i < (int)postdecisionValueAtThisSimulation.size(); i++)
    {
	//postdecisionValueAtThisSimulation[i].second -= rewardStored[i];
        double gammaN = LAMBDA + postdecisionValueAtThisSimulation[i].first.transpose() * this->postdecisionMatrixBeta * postdecisionValueAtThisSimulation[i].first,
               error = ALPHA * (this->postdecisionAttributesWeight.transpose() * postdecisionValueAtThisSimulation[i].first - postdecisionValueAtThisSimulation[i].second) + (1 - ALPHA) * (this->postdecisionAttributesWeight.transpose() * postdecisionValueAtThisSimulation[i].first - (this->predecisionAttributesWeight.transpose() * predecisionValueAtThisSimulation[i].first - rewardStored[i]));
        //cout << this->updatedAttributesWeight.transpose() * iter->first << endl;
        this->postdecisionAttributesWeight = this->postdecisionAttributesWeight - 1.0 / gammaN * this->postdecisionMatrixBeta * postdecisionValueAtThisSimulation[i].first * error;
        //cout << 1 / gammaN * this->matrixBeta << endl;
        this->postdecisionMatrixBeta = LAMBDA * (this->postdecisionMatrixBeta - 1.0 / gammaN * (this->postdecisionMatrixBeta * postdecisionValueAtThisSimulation[i].first * postdecisionValueAtThisSimulation[i].first.transpose() * this->postdecisionMatrixBeta));
        postdecisionEstimatorErrorThisSimulation += abs(error);
    }
    //cout <<  postdecisionEstimatorErrorThisSimulation << endl;
}
