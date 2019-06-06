#pragma once
#include "mdp.h"
#include "vfa.h"
#include <math.h>
#include <fstream>

#define MAX_WORK_TIME 720.0
#define CAPACITY 5
#define PENALTY_FACTOR 5
#define MAX_COST 999999.0
#define MAX_EDGE 100.0
#define UNIT_TIME 10
#define CUSTOMER_NUMBER 30
#define MAX_VEHICLE 3
#define MAX_SIMULATION 100000
#define MAX_TEST_INSTANCE 1000
#define LAG_APPROXIMATE 0
#define ROULETTE_WHEEL 1
#define MYOPIC 0

class AVI
{
public:
  void approximation(ValueFunction *valueFunction);
};
