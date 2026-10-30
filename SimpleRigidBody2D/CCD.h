#pragma once
#include "Types.h"

class CCD {
public:
  static bool findCollisionTime(const RigidBody& body, const Wall& wall, float dt, float& tc, sf::Vector2f& normal);

  static void resolveCollision(RigidBody& body, const sf::Vector2f& normal);
};
