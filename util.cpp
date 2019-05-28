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
