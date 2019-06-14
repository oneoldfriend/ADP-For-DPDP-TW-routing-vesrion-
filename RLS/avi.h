#pragma once
#include "mdp.h"
#include "vfa.h"
#include <math.h>
#include <fstream>

#define MAX_WORK_TIME 360.0
#define CAPACITY 10
#define PENALTY_FACTOR 5
#define MAX_COST 999999.0
#define MAX_EDGE 100.0
#define UNIT_TIME 5.0
#define CUSTOMER_NUMBER 10
#define MAX_VEHICLE 1
#define MAX_SIMULATION 50000
#define MAX_TEST_INSTANCE 10000
#define LAG_APPROXIMATE 0
#define ROULETTE_WHEEL 0
#define IGNORANCE_TOLERANCE 30.0
#define ASSIGNMENT 0
#define MYOPIC 1

class AVI
{
public:
  void approximation(ValueFunction *valueFunction);
};
