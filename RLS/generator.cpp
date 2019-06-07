#include "generator.h"
#include "util.h"
#include <ctime>

bool Generator::sortAscend(const pair<double, Customer *> a, const pair<double, Customer *> b)
{
    return a.first < b.first;
}

void Generator::instanceGenenrator(bool testInstanceGenerate, list<pair<double, Customer *>> *sequenceData, string fileName)
{
    list<pair<double, Customer *>> generatedCustomers;
    random_device rd;
    default_random_engine e(rd());
    double shopLocation = 25.0, serviceRange = 20.0;
    uniform_real_distribution<double> ratio(0.0, 1.0);
    uniform_real_distribution<double> shopPosX(-shopLocation, shopLocation);
    uniform_real_distribution<double> shopPosY(-shopLocation, shopLocation);
    double cancellationRatio = 0.1, hurryRatio = 0.4, timeWindowLength = (shopLocation + serviceRange), blankLength = 10.0, maxDemand = 5.0, DOD = 0.8;
    double staticCustomer = double(CUSTOMER_NUMBER) * (1 - DOD);
    int customerCount = 0;
    double staticCustomerCount = 0.0;
    while (customerCount++ < CUSTOMER_NUMBER)
    {
        Customer *customer = new Customer();
        double appearTime = ratio(e) * (MAX_WORK_TIME - timeWindowLength - blankLength);
        if (staticCustomerCount++ < staticCustomer)
        {
            customer->origin.x = shopPosX(e);
            customer->origin.y = shopPosY(e);
            uniform_real_distribution<double> customerPosX(customer->origin.x - serviceRange, customer->origin.x + serviceRange);
            uniform_real_distribution<double> customerPosy(customer->origin.y - serviceRange, customer->origin.y + serviceRange);
            customer->dest.x = customerPosX(e);
            customer->dest.y = customerPosy(e);
            customer->startTime = 0;
            customer->endTime = MAX_WORK_TIME;
            customer->weight = ratio(e) * maxDemand;
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
        customer->origin.x = shopPosX(e);
        customer->origin.y = shopPosY(e);
        uniform_real_distribution<double> customerPosX(customer->origin.x - serviceRange, customer->origin.x + serviceRange);
        uniform_real_distribution<double> customerPosy(customer->origin.y - serviceRange, customer->origin.y + serviceRange);
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
            double hurryTime = customer->startTime + ratio(e) * timeWindowLength;
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

