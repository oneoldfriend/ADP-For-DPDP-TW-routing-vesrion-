#include "mdp.h"
#include "route.h"

using namespace std;

State::State()
{
    this->currentTime = 0;
    this->currentRoute = nullptr;
}

void State::calcAttribute()
{
    this->pointSolution->calcInfo();
    this->attributes[0] = this->currentRoute->currentPos->departureTime;
    this->attributes[1] = this->notServicedCustomer.size();
    this->attributes[2] = this->pointSolution->info[1];
    this->attributes[3] = this->pointSolution->info[3];
}

void MDP::executeAction(Action a)
{
    if (a.positionToVisit != nullptr)
    {
        a.positionToVisit->prior = this->currentState.currentRoute->currentPos;
        a.positionToVisit->next = this->currentState.currentRoute->currentPos->next;
        this->currentState.currentRoute->insertOrder(a.positionToVisit);
    }
    else
    {
        this->currentState.currentRoute->currentPos->departureTime += UNIT_TIME;
        this->currentState.currentRoute->currentPos->waitTime += UNIT_TIME;
    }
    this->currentState.currentRoute->routeUpdate();
}

void MDP::undoAction(Action a)
{
    if (a.positionToVisit != nullptr)
    {
        this->currentState.currentRoute->removeOrder(a.positionToVisit);
    }
    else
    {
        this->currentState.currentRoute->currentPos->departureTime -= UNIT_TIME;
        this->currentState.currentRoute->currentPos->waitTime -= UNIT_TIME;
    }
    this->currentState.currentRoute->routeUpdate();
}

void MDP::integerToAction(int actionNum, State S, Action *a)
{
    //根据当前动作号进行二进制转换为具体动作
    if (actionNum >= 0)
    {
        a->positionToVisit = S.reachableCustomer[actionNum];
    }
    else
    {
        a->positionToVisit = nullptr;
    }
}

void MDP::findBestAction(Action *a, ValueFunction valueFunction, double *reward)
{
    int actionNum = 0, maxActionNum = this->currentState.reachableCustomer.size(), bestActionNum = -1;
    double bestActionValue = MAX_COST;
    while (actionNum < maxActionNum)
    {
        //检查每个可能动作的可行性并对可行动作进行评估
        Action tempAction;
        double actionValue = 0;
        this->integerToAction(actionNum, this->currentState, &tempAction);
        double immediateReward = 0;
        if (this->checkActionFeasibility(tempAction, &immediateReward))
        {
            //若动作可行，则进行评估
            actionValue = immediateReward + valueFunction.getValue(this->currentState, tempAction);
            if (actionValue < bestActionValue)
            {
                //记录更优的动作
                *reward = immediateReward;
                bestActionValue = actionValue;
                bestActionNum = actionNum;
            }
        }
        //回撤动作继续下一个评估
        this->undoAction(tempAction);
        actionNum++;
    }
    this->integerToAction(bestActionNum, this->currentState, a);
    if (bestActionNum == -1)
    {
        this->checkActionFeasibility(*a, reward);
        this->undoAction(*a);
    }
}

bool MDP::checkActionFeasibility(Action a, double *reward)
{
    double currentCost = this->currentState.currentRoute->cost;
    this->executeAction(a);
    bool feasibility = this->currentState.currentRoute->checkFeasibility();
    double newCost = this->currentState.currentRoute->cost;
    *reward = newCost - currentCost;
    return feasibility;
}

MDP::MDP(string fileName)
{
    this->solution = Solution();
    ifstream trainFile(fileName, ios::in);
    double appearTime = 0;
    while (!trainFile.eof())
    {
        //读取instance数据
        trainFile >> appearTime;
        Customer *customer = new Customer();
        trainFile >> customer->id;
        trainFile >> customer->origin.x;
        trainFile >> customer->origin.y;
        trainFile >> customer->dest.x;
        trainFile >> customer->dest.y;
        trainFile >> customer->startTime;
        trainFile >> customer->endTime;
        trainFile >> customer->weight;
        trainFile >> customer->priority;
        this->sequenceData.push_back(make_pair(appearTime, customer));
    }
    auto last = this->sequenceData.rbegin();
    delete last->second;
    this->sequenceData.pop_back();
    trainFile.close();
    this->currentState = State();
    for (auto iter = this->sequenceData.begin(); iter != this->sequenceData.end(); ++iter)
    {
        if (iter->first == 0.0)
        {
            //加入提前已知的顾客
            this->currentState.notServicedCustomer.push_back(new Order(iter->second, true));
        }
        else
        {
            break;
        }
    }
    //状态当前车辆初始化为第一辆车
    this->currentState.currentRoute = &this->solution.routes[0];
    this->currentState.pointSolution = &this->solution;
    for (auto iter = this->currentState.notServicedCustomer.begin(); iter != this->currentState.notServicedCustomer.end(); ++iter)
    {
        this->currentState.reachableCustomer.push_back(*iter);
    }
}

double MDP::reward(State S, Action a)
{
    //复制当前解并对副本执行动作计算立即反馈
    double currentCost = S.currentRoute->cost;
    this->executeAction(a);
    double newCost = S.currentRoute->cost;
    this->undoAction(a);
    return newCost - currentCost;
}

void MDP::transition(Action a)
{
    //执行动作
    double lastDecisionTime = this->currentState.currentTime;
    this->executeAction(a);
    //更新当前状态
    if (a.positionToVisit != nullptr)
    {
        //若执行车辆去某个订单的起点取货
        if (a.positionToVisit->isOrigin == true)
        {
            //将订单终点加入未服务位置集中
            this->currentState.notServicedCustomer.push_back(new Order(a.positionToVisit->customer, false));
        }
        //在未服务位置集中删除该位置
        for (auto iter = this->currentState.notServicedCustomer.begin(); iter != this->currentState.notServicedCustomer.end(); ++iter)
        {
            if (*iter == a.positionToVisit)
            {
                this->currentState.notServicedCustomer.erase(iter);
                break;
            }
        }
        //更新当前路径的available position为下一个点
        this->currentState.currentRoute->currentPos = this->currentState.currentRoute->currentPos->next;
    }
    else
    {
        //若选择原地待命则检查车辆是否能原地待命或者返回仓库结束配送
        if (this->currentState.currentRoute->tail->departureTime + UNIT_TIME > MAX_WORK_TIME)
        {
            //若车辆不能原地等待则直接返回仓库结束配送
            this->undoAction(a);
            this->currentState.currentRoute->currentPos = this->currentState.currentRoute->tail;
        }
    }
    this->currentState.currentTime = MAX_WORK_TIME;
    this->currentState.currentRoute = nullptr;
    //找到下一辆空闲车辆,更新状态相关信息
    for (auto iter = this->solution.routes.begin(); iter != this->solution.routes.end(); ++iter)
    {
        //检查车辆是否已经结束配送
        if (iter->currentPos == iter->tail)
        {
            continue;
        }
        else if (this->currentState.currentTime > iter->currentPos->departureTime)
        {
            this->currentState.currentTime = iter->currentPos->departureTime;
            this->currentState.currentRoute = &*iter;
        }
    }
    //观察新顾客信息
    this->observation(lastDecisionTime);
}

void MDP::observation(double lastDecisionTime)
{
    for (auto sequenceIter = this->sequenceData.begin(); sequenceIter != this->sequenceData.end();)
    {
        if (sequenceIter->first > this->currentState.currentTime)
        {
            break;
        }
        //找到上次决策时间到当前决策时间中产生的新顾客信息
        if (sequenceIter->first > lastDecisionTime && sequenceIter->first <= this->currentState.currentTime)
        {
            if (sequenceIter->second->priority == 1)
            {
                //若有新顾客产生，则将其加入到待插入顾客集中
                this->customers[sequenceIter->second->id] = sequenceIter->second;
                this->currentState.notServicedCustomer.push_back(new Order(sequenceIter->second, true));
            }
            else
            {
                //若观察到顾客退单或催单，则直接对原有顾客信息进行更新
                this->customers[sequenceIter->second->id]->priority = sequenceIter->second->priority;
                delete sequenceIter->second;
            }
        }
        this->sequenceData.erase(sequenceIter++);
    }
    //更新当前车辆可以合法访问的点
    this->currentState.reachableCustomer.clear();
    if (this->currentState.currentRoute != nullptr)
    {
        for (auto iter = this->currentState.notServicedCustomer.begin(); iter != this->currentState.notServicedCustomer.end(); ++iter)
        {
            if ((*iter)->isOrigin)
            {
                this->currentState.reachableCustomer.push_back(*iter);
            }
            else
            {
                PointOrder p = this->currentState.currentRoute->head->next;
                while (p != this->currentState.currentRoute->tail)
                {
                    if (p->customer->id == (*iter)->customer->id)
                    {
                        this->currentState.reachableCustomer.push_back(*iter);
                        break;
                    }
                    else
                    {
                        p = p->next;
                    }
                }
            }
        }
    }

    //update the solution(delete the customers with cancellation)
    /* 
    for (auto routeIter = this->solution.routes.begin(); routeIter != this->solution.routes.end(); ++routeIter)
    {
        PointOrder p = routeIter->currentPos->next;
        bool cancellation = false;
        while (p != routeIter->tail)
        {
            //检查若有顾客退单，则将其从解中删除
            if (p->customer->priority == 0)
            {
                cancellation = true;
                PointOrder tempPtr = p->next;
                routeIter->removeOrder(p);
                delete p;
                p = tempPtr;
            }
            else
            {
                p = p->next;
            }
        }
        if (cancellation)
        {
            //若有顾客退单导致解的变化，则更新对应路径的信息
            routeIter->routeUpdate();
        }
    }*/
}