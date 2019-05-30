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
#define CUSTOMER_NUMBER 10
#define MAX_VEHICLE 3
#define MAX_SIMULATION 50000
#define MAX_TEST_INSTANCE 100
#define LAG_APPROXIMATE 0
#define MYOPIC 0

class AVI
{
public:
  void approximation(ValueFunction *valueFunction);
};
