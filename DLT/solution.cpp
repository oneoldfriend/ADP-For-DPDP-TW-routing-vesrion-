#include "solution.h"
#include "avi.h"
#include "vfa.h"

Solution::Solution()
{
    for (int i = 0; i < MAX_VEHICLE; i++)
    {
        PointRoute route = new Route();
        this->routes.push_back(*route);
    }
    this->cost = 0;
    this->waitTime = 0;
    this->travelTime = 0;
    this->penalty = 0;
}

double Solution::calcCost()
{
    double cost = 0;
    for (auto iter = this->routes.begin(); iter != this->routes.end(); ++iter)
    {
        this->cost += iter->cost;
        this->penalty += iter->penalty;
        this->travelTime += iter->travelTime;
        this->waitTime += iter->waitTime;
    }
    return cost;
}

void Solution::solutionCopy(Solution *source)
{
    auto thisRouteIter = this->routes.begin();
    auto sourceRouteIter = source->routes.begin();
    for (; thisRouteIter != this->routes.end(); ++thisRouteIter, ++sourceRouteIter)
    {
        thisRouteIter->routeCopy(*sourceRouteIter);
    }
    this->cost = source->cost;
}

void Solution::solutionDelete()
{
    for (auto iter = this->routes.begin(); iter != this->routes.end(); ++iter)
    {
        iter->deleteRoute();
    }
}


/*bool Solution::greedyInsertion(Action a)
{
    vector<Customer *> customerToBeInserted;
    //获得需要插入的顾客信息
    for (auto iter = a.customerConfirmation.begin(); iter != a.customerConfirmation.end(); ++iter)
    {
        if (iter->second)
        {
            customerToBeInserted.push_back(iter->first);
        }
    }
    for (auto customerIter = customerToBeInserted.begin(); customerIter != customerToBeInserted.end(); ++customerIter)
    {
        //找到每个顾客在解中的最优位置进行插入
        bool feasibility = false;
        PointOrder origin = new Order(*customerIter, true);
        PointOrder dest = new Order(*customerIter, false);
        pair<PointOrder, PointOrder> bestOriginPos, bestDestPos;
        PointRoute bestRoute = nullptr;
        double bestCost = MAX_COST;
        for (auto routeIter = this->routes.begin(); routeIter != this->routes.end(); ++routeIter)
        {
            double oldCost = bestCost;
            //找到顾客在当前路径中的最优位置
            if (routeIter->findBestPosition(origin, dest, &bestCost))
            {
                //若存在合法位置
                feasibility = true;
                if (oldCost > bestCost)
                {
                    bestOriginPos.first = origin->prior;
                    bestOriginPos.second = origin->next;
                    bestDestPos.first = dest->prior;
                    bestDestPos.second = dest->next;
                    bestRoute = &(*routeIter);
                }
            }
        }
        if (feasibility == false)
        {
            //若有一个顾客无法在当前解中找到合法位置，则该动作非法，并返回合法性
            delete origin;
            delete dest;
            return feasibility;
        }
        else
        {
            //否则将插入顾客最优位置
            origin->prior = bestOriginPos.first;
            origin->next = bestOriginPos.second;
            dest->prior = bestDestPos.first;
            dest->next = bestDestPos.second;
            bestRoute->insertOrder(dest);
            bestRoute->insertOrder(origin);
            bestRoute->routeUpdate();
        }
    }
    this->cost = this->calcCost();
    return true;
}*/


/*void Solution::calcAttribute()
{
    this->attribute[0] = MAX_EDGE;
    int deliveredWeights = 0, availableVehicle = 0;
    double availableTime = 0.0;
    for (auto routeIter = this->routes.begin(); routeIter != this->routes.end(); routeIter++)
    {
        availableTime += MAX_WORK_TIME - routeIter->tail->arrivalTime;
        if (routeIter->tail->arrivalTime + MAX_EDGE <= MAX_WORK_TIME)
        {
            availableVehicle++;
        }
        PointOrder p = routeIter->head;
        while (p != routeIter->currentPos)
        {
            if (!p->isOrigin)
            {
                deliveredWeights++;
            }
            p = p->next;
        }
    }
    this->attribute[1] = ceil(availableTime / MAX_EDGE);
    this->attribute[2] = deliveredWeights;
    this->attribute[3] = availableVehicle;
}*/