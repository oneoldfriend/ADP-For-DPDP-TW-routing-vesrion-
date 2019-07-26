#pragma once
#include "mdp.h"
#include <math.h>
#include <fstream>

class ValueFunction;

#define MAX_WORK_TIME 720.0
#define CAPACITY 140
#define PENALTY_FACTOR 5
#define DEGREE_OF_DYNAMISM 0.8
#define MAX_COST 999999.0
#define MAX_EDGE 100.0
#define UNIT_TIME 5.0
#define SPEED 20
#define CUSTOMER_NUMBER 30
#define MAX_VEHICLE 1
#define MAX_SIMULATION 100000
#define MAX_TEST_INSTANCE 10000
#define LAG_APPROXIMATE 50
#define ASSIGNMENT 0
#define IGNORANCE_TOLERANCE 30.0
#define MYOPIC 0

class AVI
{
public:
  void approximation(ValueFunction *valueFunction);
};
