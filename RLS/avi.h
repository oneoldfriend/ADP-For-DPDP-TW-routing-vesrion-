#pragma once
#include "mdp.h"
#include "vfa.h"
#include <math.h>
#include <fstream>

#define MAX_WORK_TIME 360.0
#define CAPACITY 5
#define PENALTY_FACTOR 5
#define MAX_COST 999999.0
#define MAX_EDGE 100.0
#define UNIT_TIME 10
#define CUSTOMER_NUMBER 30
#define MAX_VEHICLE 3
#define MAX_SIMULATION 1000000
#define MAX_TEST_INSTANCE 10000
#define LAG_APPROXIMATE 50000
#define ROULETTE_WHEEL 0
#define IGNORANCE_TOLERANCE 30.0
#define ASSIGNMENT 0
#define MYOPIC 0

class AVI
{
public:
  void approximation(ValueFunction *valueFunction);
};
