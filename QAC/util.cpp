#include "util.h"

void Util::infoCopy(Customer *target, Customer *source)
{
    target->dest.x = source->dest.x;
    target->dest.y = source->dest.y;
    target->origin.x = source->origin.x;
    target->origin.y = source->origin.y;
    target->startTime = source->startTime;
    target->endTime = source->endTime;
    target->weight = source->weight;
    target->priority = source->priority;
    target->id = source->id;
}

double Util::standardDeviation(vector<double> sample)
{
    double sum = 0, mean = 0, variance = 0;
    for (auto iter = sample.begin(); iter != sample.end(); ++iter)
    {
        sum += *iter;
    }
    mean = sum / sample.size();
    for (auto iter = sample.begin(); iter != sample.end(); ++iter)
    {
        variance += pow(*iter - mean, 2);
    }
    variance = variance / (sample.size() - 1);
    return sqrt(variance);
}

double Util::calcTravelTime(Position a, Position b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

int Util::softmax(map<int, double> data, Eigen::Vector4d *scoreExpectation, map<int, Eigen::Vector4d> actionScoreSample)
{
    double maxData = data.begin()->second;
    for (auto iter = data.begin(); iter != data.end(); ++iter)
    {
        if (maxData < iter->second)
        {
            maxData = iter->second;
        }
    }
    double denominator = 0;
    for (auto iter = data.begin(); iter != data.end(); ++iter)
    {
        iter->second = exp(iter->second - maxData);
        denominator += iter->second;
    }
    int chosenOne = -1;
    double prob = rand() / double(RAND_MAX), cumProb = 0.0;
    for (auto iter = data.begin(); iter != data.end(); ++iter)
    {
        cumProb += iter->second / denominator;
        *scoreExpectation += iter->second / denominator * actionScoreSample[iter->first];
        if (prob <= cumProb)
        {
            chosenOne = iter->first;
            prob = 2.0;
        }
    }
    return chosenOne;
}