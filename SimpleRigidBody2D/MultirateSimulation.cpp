#include "MultirateSimulation.h"
#include <cmath>
#include <algorithm>

MultirateSimulation::MultirateSimulation(float targetTimeStep)
  : accumulator(0.0f), timeStep(targetTimeStep) {
}

void MultirateSimulation::update(std::vector<RigidBody>& bodies, const std::vector<Wall>& walls, float frameTime, int& stepsCount) {
  accumulator += frameTime;
  stepsCount = 0;

  while (accumulator >= timeStep) {
    stepsCount++;
    for (size_t i = 0; i < bodies.size(); ++i) {
      if (bodies[i].isStatic) continue;

      bodies[i].position += bodies[i].velocity * timeStep;

      for (size_t j = 0; j < bodies.size(); ++j) {
        if (i == j || !bodies[j].isStatic) continue;

        sf::Vector2f delta = bodies[i].position - bodies[j].position;
        float distSq = delta.x * delta.x + delta.y * delta.y;
        float minDist = bodies[i].radius + bodies[j].radius;

        if (distSq < minDist * minDist) {
          float dist = std::sqrt(distSq);
          sf::Vector2f normal = (dist > 0.0f) ? delta / dist : sf::Vector2f(1.0f, 0.0f);

          float overlap = minDist - dist;
          bodies[i].position += normal * overlap;

          float dotProduct = bodies[i].velocity.x * normal.x + bodies[i].velocity.y * normal.y;
          if (dotProduct < 0.0f) {
            bodies[i].velocity -= (1.0f + RESTITUTION) * dotProduct * normal;
          }
        }
      }

      for (const auto& wall : walls) {
        float wLeft = wall.rect.position.x;
        float wRight = wall.rect.position.x + wall.rect.size.x;
        float wTop = wall.rect.position.y;
        float wBottom = wall.rect.position.y + wall.rect.size.y;

        if (bodies[i].position.x - bodies[i].radius < wRight &&
          bodies[i].position.x + bodies[i].radius > wLeft &&
          bodies[i].position.y - bodies[i].radius < wBottom &&
          bodies[i].position.y + bodies[i].radius > wTop) {

          float overlapX = 0.0f;
          float overlapY = 0.0f;
          sf::Vector2f normal(0.0f, 0.0f);

          float midX = bodies[i].position.x;
          float distToLeft = std::abs(midX + bodies[i].radius - wLeft);
          float distToRight = std::abs(wRight - (midX - bodies[i].radius));

          if (distToLeft < distToRight) {
            overlapX = distToLeft;
            normal.x = -1.0f;
          }
          else {
            overlapX = distToRight;
            normal.x = 1.0f;
          }

          float midY = bodies[i].position.y;
          float distToTop = std::abs(midY + bodies[i].radius - wTop);
          float distToBottom = std::abs(wBottom - (midY - bodies[i].radius));

          if (distToTop < distToBottom) {
            overlapY = distToTop;
            normal.y = -1.0f;
          }
          else {
            overlapY = distToBottom;
            normal.y = 1.0f;
          }

          if (overlapX < overlapY) {
            bodies[i].position.x += normal.x * overlapX;
            float dot = bodies[i].velocity.x * normal.x;
            if (dot < 0.0f) {
              bodies[i].velocity.x -= (1.0f + RESTITUTION) * dot * normal.x;
            }
          }
          else {
            bodies[i].position.y += normal.y * overlapY;
            float dot = bodies[i].velocity.y * normal.y;
            if (dot < 0.0f) {
              bodies[i].velocity.y -= (1.0f + RESTITUTION) * dot * normal.y;
            }
          }
        }
      }
    }
    accumulator -= timeStep;
  }
}
