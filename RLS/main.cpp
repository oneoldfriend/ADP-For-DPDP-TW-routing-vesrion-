#include "solver.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>

double e_d(double lambda)
{
    float z;
    do
    {
        z = ((float)rand()) / RAND_MAX;
    } while (z == 0 || z == 1);
    return -(1 / lambda) * log(z);
}

int main()
{
    double lambda = 30.0 / 720.0, i = 0.0, count = 0.0;
    srand((unsigned int)time(NULL));
    while (i < 720.0)
    {
        i += e_d(lambda);
        cout << "No. : " << ++count << "  Current Time: " << i << endl;
    }
    Solver solver;
    solver.solve();
}