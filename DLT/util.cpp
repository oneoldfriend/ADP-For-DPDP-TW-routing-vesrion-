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

double Util::standardDeviation(double *sample, int size)
{
    double sum = 0, mean = 0, variance = 0;
    for (int i = 0; i < size; i++)
    {
        sum += sample[i];
    }
    mean = sum / size;
    for (int i = 0; i < size; i++)
    {
        variance += pow(sample[i] - mean, 2);
    }
    variance = variance / (size - 1);
    return sqrt(variance);
}

double Util::calcTravelTime(Position a, Position b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2)) / SPEED * 60.0;
}
