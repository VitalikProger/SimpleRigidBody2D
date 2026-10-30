#include "CCD.h"
#include <cmath>

bool CCD::findCollisionTime(const RigidBody& body, const Wall& wall, float dt, float& tc, sf::Vector2f& normal) {
  if (body.isStatic || (body.velocity.x == 0.0f && body.velocity.y == 0.0f)) return false;

  float wallLeft = wall.rect.position.x;
  float wallRight = wall.rect.position.x + wall.rect.size.x;
  float wallTop = wall.rect.position.y;
  float wallBottom = wall.rect.position.y + wall.rect.size.y;

  float t_hit = -1.0f;
  sf::Vector2f temp_normal(0.0f, 0.0f);

  if (body.velocity.x > 0.0f) {
    t_hit = (wallLeft - (body.position.x + body.radius)) / body.velocity.x;
    temp_normal = sf::Vector2f(-1.0f, 0.0f);
  }
  else if (body.velocity.x < 0.0f) {
    t_hit = ((wallRight + body.radius) - body.position.x) / body.velocity.x;
    temp_normal = sf::Vector2f(1.0f, 0.0f);
  }

  if (t_hit >= 0.0f && t_hit <= dt) {
    float y_at_hit = body.position.y + body.velocity.y * t_hit;
    if (y_at_hit + body.radius >= wallTop && y_at_hit - body.radius <= wallBottom) {
      tc = t_hit;
      normal = temp_normal;
      return true;
    }
  }

  if (body.velocity.y > 0.0f) {
    t_hit = (wallTop - (body.position.y + body.radius)) / body.velocity.y;
    temp_normal = sf::Vector2f(0.0f, -1.0f);
  }
  else if (body.velocity.y < 0.0f) {
    t_hit = ((wallBottom + body.radius) - body.position.y) / body.velocity.y;
    temp_normal = sf::Vector2f(0.0f, 1.0f);
  }

  if (t_hit >= 0.0f && t_hit <= dt) {
    float x_at_hit = body.position.x + body.velocity.x * t_hit;
    if (x_at_hit + body.radius >= wallLeft && x_at_hit - body.radius <= wallRight) {
      tc = t_hit;
      normal = temp_normal;
      return true;
    }
  }

  return false;
}

void CCD::resolveCollision(RigidBody& body, const sf::Vector2f& normal) {
  float dotProduct = body.velocity.x * normal.x + body.velocity.y * normal.y;
  body.velocity -= (1.0f + RESTITUTION) * dotProduct * normal;
}
