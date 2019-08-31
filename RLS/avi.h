#pragma once
#include "mdp.h"
#include "vfa.h"
#include <math.h>
#include <fstream>

#define MAX_WORK_TIME 720.0
#define CAPACITY 140
#define PENALTY_FACTOR 5
#define DEGREE_OF_DYNAMISM 0.8
#define MAX_COST 999999.0
#define MAX_EDGE 100.0
#define UNIT_TIME 5.0
#define SPEED 20
#define CUSTOMER_NUMBER 20
#define MAX_VEHICLE 1
#define MAX_SIMULATION 50000
#define MAX_TEST_INSTANCE 10000
#define LAG_APPROXIMATE 10
#define GENERATOR 0
#define ASSIGNMENT_MYOPIC 0
#define ROUTING_MYOPIC 0

class AVI
{
public:
  void approximation(ValueFunction *valueFunction);
};
