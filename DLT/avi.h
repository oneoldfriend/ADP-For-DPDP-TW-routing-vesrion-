#pragma once
#define MAX_WORK_TIME 360.0
#define CAPACITY 5
#define PENALTY_FACTOR 5
#define MAX_COST 999999.0
#define MAX_EDGE 100.0
#define UNIT_TIME 10
#define CUSTOMER_NUMBER 20
#define MAX_VEHICLE 2
#define MAX_SIMULATION 50000
#define MAX_TEST_INSTANCE 1000
#define LAG_APPROXIMATE 0
#define ROULETTE_WHEEL 1
#define MYOPIC 1
#include "mdp.h"
#include "vfa.h"
#include <math.h>
#include <fstream>


class AVI
{
public:
  void approximation(ValueFunction *valueFunction);
};
