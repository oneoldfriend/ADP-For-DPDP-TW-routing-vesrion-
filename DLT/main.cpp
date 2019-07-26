#include "solver.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>

int main()
{
    clock_t start, end;
    start = clock();
    Solver solver;
    solver.solve();
    end = clock();
    cout << "runtime: " << double(end - start) / CLOCKS_PER_SEC / 60.0 << " min" << endl;
}
