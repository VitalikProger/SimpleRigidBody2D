#pragma once
#include "Types.h"
#include <vector>

class MultirateSimulation {
private:
  float accumulator;
  float timeStep;

public:
  MultirateSimulation(float targetTimeStep);

  void update(std::vector<RigidBody>& bodies, const std::vector<Wall>& walls, float frameTime, int& stepsCount);
};
