#pragma once
#include <SFML/Graphics.hpp>

const float DT_60 = 1.0f / 60.0f;
const float DT_240 = 1.0f / 240.0f;
const float RESTITUTION = 0.85f;

enum class SimulationMode {
  Discrete_60Hz,
  CCD_60Hz,
  MultiRate_240Hz
};

struct RigidBody {
  sf::Vector2f position;
  sf::Vector2f velocity;
  float radius;
  bool isStatic;
  sf::Color color;
};

struct Wall {
  sf::FloatRect rect;
  sf::Color color;
};
