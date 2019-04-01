#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H
#include <iostream>
#include <vector>
#include <math.h>
#include "vehicle.h"

using namespace std;

class CostFunction {
public:
  const double COLLISION  = pow(10,7);
  const double DANGER     = pow(10,5);
  const double REACH_GOAL = pow(10,5);
  const double COMFORT    = .65 * pow(10,5);
  const double EFFICIENCY = 2 * pow(10,3);//2

  const double DESIRED_BUFFER = 1;//1.5;
  const double PLANNING_HORIZON = 2;

  vector<vector<double>> sensor_fusion;
  Vehicle *vehicle;

  /**
   * Constructor
   */
  CostFunction(Vehicle *v, vector<vector<double>> s);

  /*
   * Cost functions
   */
  double ChangeLane();
  double DistanceGoalLane();
  double Inefficiency();
  double Collision();
  double Buffer();
  double Target();
  double Compute();
  
};

#endif
