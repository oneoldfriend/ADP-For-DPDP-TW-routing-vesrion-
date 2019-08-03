#include "generator.h"
#include "util.h"
#include <ctime>
#include <algorithm>

bool Generator::sortAscend(const pair<double, Customer *> a, const pair<double, Customer *> b)
{
    return a.first < b.first;
}

void Generator::instanceGenenrator(bool testInstanceGenerate, list<pair<double, Customer *>> *sequenceData, string fileName)
{
    list<pair<double, Customer *>> generatedCustomers;
    random_device rd;
    default_random_engine e(rd());
    double shopLocation = 10.0, serviceRange = 5.0;
    uniform_real_distribution<double> ratio(0.0, 1.0);
    double cancellationRatio = 0.1, hurryRatio = 0.4, timeWindowLength = 60.0, blankLength = 10.0, maxDemand = 5.0;
    double staticCustomer = double(CUSTOMER_NUMBER) * (1 - DEGREE_OF_DYNAMISM);
    int customerCount = 0;
    double staticCustomerCount = 0.0;
    double cluster[3][4] = {{-10.0, 0.0, 0.0, 10.0}, {0.0, 10.0, -10.0, 0.0}, {0.0, 10.0, 0.0, 10.0}};
    int clusterNum = 3;
    int clusterIndex = 0, clusterCount = 0, clusterLimit = CUSTOMER_NUMBER - (int)staticCustomer / clusterNum;
    double lambda = (double)CUSTOMER_NUMBER / ((double)MAX_WORK_TIME - timeWindowLength - blankLength), i = 0.0;
    vector<double> apStore;
    while (i < (MAX_WORK_TIME - timeWindowLength - blankLength))
    {
        i += -(1 / lambda) * log(ratio(e));
        apStore.push_back(i);
    }
    while (customerCount++ < CUSTOMER_NUMBER)
    {
        /*if (customerCount > apStore.size())
        {
            break;
        }*/
        Customer *customer = new Customer();
        normal_distribution<double> ap(360, 120);
        //double appearTime = (MAX_WORK_TIME - timeWindowLength - blankLength) * ap(e);
	    //cout << appearTime << endl;
        //double appearTime = apStore[customerCount - 1];
        double appearTime = ratio(e) * (MAX_WORK_TIME - timeWindowLength - blankLength);
        if (staticCustomerCount++ < staticCustomer)
        {
            customer->origin.x = 0.0;
            customer->origin.y = 0.0;
            uniform_real_distribution<double> customerPosX(-shopLocation, shopLocation);
            uniform_real_distribution<double> customerPosy(-shopLocation, shopLocation);
            customer->dest.x = customerPosX(e);
            customer->dest.y = customerPosy(e);
            customer->startTime = 0;
            customer->endTime = MAX_WORK_TIME;
            customer->weight = ratio(e) * maxDemand * 10.0;
            char idString[] = {char(customerCount / 1000 + 48), char(customerCount % 1000 / 100 + 48),
                               char(customerCount % 100 / 10 + 48), char(customerCount % 10 + 48), '\0'};
            customer->id = idString;
            if (!testInstanceGenerate)
            {
                sequenceData->push_back(make_pair(0, customer));
            }
            else
            {
                generatedCustomers.push_back(make_pair(0, customer));
            }
            continue;
        }
        if (clusterIndex + 1 < clusterNum && clusterCount > clusterLimit)
        {
            clusterIndex++;
        }
        clusterCount++;
        uniform_real_distribution<double> shopPosX(-shopLocation, shopLocation);
        uniform_real_distribution<double> shopPosY(-shopLocation, shopLocation);
        customer->origin.x = shopPosX(e);
        customer->origin.y = shopPosY(e);
        /*uniform_real_distribution<double> customerPosX(max(-shopLocation, customer->origin.x - serviceRange),
                                                       min(shopLocation, customer->origin.x + serviceRange));
        uniform_real_distribution<double> customerPosy(max(-shopLocation, customer->origin.y - serviceRange),
                                                       min(shopLocation, customer->origin.y + serviceRange));*/
        uniform_real_distribution<double> customerPosX(-shopLocation, shopLocation);
        uniform_real_distribution<double> customerPosy(-shopLocation, shopLocation);
        customer->dest.x = customerPosX(e);
        customer->dest.y = customerPosy(e);
        customer->startTime = appearTime + blankLength;
        customer->endTime = customer->startTime + timeWindowLength;
        customer->weight = ratio(e) * maxDemand;
        char idString[] = {char(customerCount / 1000 + 48), char(customerCount % 1000 / 100 + 48),
                           char(customerCount % 100 / 10 + 48), char(customerCount % 10 + 48), '\0'};
        customer->id = idString;
        if (!testInstanceGenerate)
        {
            sequenceData->push_back(make_pair(appearTime, customer));
        }
        else
        {
            generatedCustomers.push_back(make_pair(appearTime, customer));
        }
        double isCanceled = ratio(e), isHurry = ratio(e);
        if (isCanceled <= cancellationRatio)
        {
            Customer *cancel = new Customer();
            Util::infoCopy(cancel, customer);
            cancel->priority = 0;
            double cancelTime = customer->startTime - blankLength + ratio(e) * blankLength;
            if (!testInstanceGenerate)
            {
                sequenceData->push_back(make_pair(cancelTime, cancel));
            }
            else
            {
                generatedCustomers.push_back(make_pair(cancelTime, cancel));
            }
        }
        if (isHurry <= hurryRatio)
        {
            Customer *hurry = new Customer();
            Util::infoCopy(hurry, customer);
            hurry->priority = 2;
            double hurryTime = customer->startTime + timeWindowLength / 2.0 + ratio(e) * timeWindowLength / 2.0;
            if (!testInstanceGenerate)
            {
                sequenceData->push_back(make_pair(hurryTime, hurry));
            }
            else
            {
                generatedCustomers.push_back(make_pair(hurryTime, hurry));
            }
        }
    }
    if (!testInstanceGenerate)
    {
        sequenceData->sort(sortAscend);
    }
    else
    {
        generatedCustomers.sort(sortAscend);
    }
    if (testInstanceGenerate)
    {
        ofstream outFile(fileName, ios::out);
        for (auto iter = generatedCustomers.begin(); iter != generatedCustomers.end(); ++iter)
        {
            outFile << iter->first << " ";
            outFile << iter->second->id << " ";
            outFile << iter->second->origin.x << " ";
            outFile << iter->second->origin.y << " ";
            outFile << iter->second->dest.x << " ";
            outFile << iter->second->dest.y << " ";
            outFile << iter->second->startTime << " ";
            outFile << iter->second->endTime << " ";
            outFile << iter->second->weight << " ";
            outFile << iter->second->priority << endl;
            delete iter->second;
        }
        outFile.close();
    }
}
