#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <string>
#include <iomanip>
#include <cmath>
#include "Types.h"
#include "CCD.h"
#include "MultirateSimulation.h"

int main() {
  sf::RenderWindow window(sf::VideoMode({ 1000, 700 }), "ITMO Phys Engine: CCD vs Multi-rate");
  window.setFramerateLimit(60);

  sf::Font font;
  if (!font.openFromFile("arial.ttf")) {
    std::cerr << "Warning: Could not load arial.ttf!" << std::endl;
  }

  sf::Text uiText(font);
  uiText.setString("");
  uiText.setCharacterSize(15);
  uiText.setFillColor(sf::Color::Red);
  uiText.setPosition({ 15.0f, 15.0f });

  SimulationMode mode = SimulationMode::Discrete_60Hz;
  MultirateSimulation multiRateSystem(DT_240);

  std::vector<Wall> walls;
  walls.push_back({ sf::FloatRect({500.0f, 150.0f}, {10.0f, 400.0f}), sf::Color::Red });
  walls.push_back({ sf::FloatRect({50.0f, 50.0f},   {900.0f, 20.0f}), sf::Color::White });
  walls.push_back({ sf::FloatRect({50.0f, 630.0f},  {900.0f, 20.0f}), sf::Color::White });
  walls.push_back({ sf::FloatRect({50.0f, 50.0f},   {20.0f, 600.0f}), sf::Color::White });
  walls.push_back({ sf::FloatRect({930.0f, 50.0f},  {20.0f, 600.0f}), sf::Color::White });

  std::vector<RigidBody> initial_bodies;

  RigidBody projectile;
  projectile.position = sf::Vector2f(100.0f, 350.0f);
  projectile.velocity = sf::Vector2f(2000.0f, 0.0f);
  projectile.radius = 5.0f;
  projectile.isStatic = false;
  projectile.color = sf::Color::Yellow;
  initial_bodies.push_back(projectile);

  initial_bodies.push_back({ sf::Vector2f(300.0f, 220.0f), sf::Vector2f(0.0f, 0.0f), 40.0f, true, sf::Color(128,128,128) });
  initial_bodies.push_back({ sf::Vector2f(700.0f, 480.0f), sf::Vector2f(0.0f, 0.0f), 40.0f, true, sf::Color(128,128,128) });
  initial_bodies.push_back({ sf::Vector2f(350.0f, 480.0f), sf::Vector2f(0.0f, 0.0f), 40.0f, true, sf::Color(128,128,128) });

  std::mt19937 rng(42);
  std::uniform_real_distribution<float> posX(120.0f, 440.0f);
  std::uniform_real_distribution<float> posY(100.0f, 600.0f);
  std::uniform_real_distribution<float> vel(-150.0f, 150.0f);

  for (int i = 0; i < 25; ++i) {
    sf::Color bodyColor = sf::Color::Cyan;
    if (i == 0) {
      bodyColor = sf::Color(147, 112, 219);
    }
    initial_bodies.push_back({ sf::Vector2f(posX(rng), posY(rng)), sf::Vector2f(vel(rng), vel(rng)), 20.0f, false, bodyColor });
  }

  std::vector<RigidBody> bodies = initial_bodies;
  sf::Clock deltaClock;
  sf::Clock logClock;
  double simTimeMs = 0.0;
  int stepsCount = 0;

  std::cout << "========================================================\n";
  std::cout << "  ITMO Physics Engine Console Log Initialized\n";
  std::cout << "  Purple Circle tracked as Object #4\n";
  std::cout << "========================================================\n\n";

  while (window.isOpen()) {
    while (const std::optional<sf::Event> event = window.pollEvent()) {
      if (event->is<sf::Event::Closed>()) {
        window.close();
      }
      if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {
        if (keyPressed->code == sf::Keyboard::Key::Num1) { mode = SimulationMode::Discrete_60Hz; bodies = initial_bodies; }
        if (keyPressed->code == sf::Keyboard::Key::Num2) { mode = SimulationMode::CCD_60Hz; bodies = initial_bodies; }
        if (keyPressed->code == sf::Keyboard::Key::Num3) { mode = SimulationMode::MultiRate_240Hz; bodies = initial_bodies; }
        if (keyPressed->code == sf::Keyboard::Key::R) { bodies = initial_bodies; }
      }
    }

    float frameTime = deltaClock.restart().asSeconds();
    if (frameTime > 0.1f) frameTime = 0.1f;

    auto start = std::chrono::high_resolution_clock::now();

    if (mode == SimulationMode::Discrete_60Hz) {
      stepsCount = 1;
      for (size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i].isStatic) continue;
        bodies[i].position += bodies[i].velocity * DT_60;

        for (size_t j = 0; j < bodies.size(); ++j) {
          if (i == j || !bodies[j].isStatic) continue;
          sf::Vector2f delta = bodies[i].position - bodies[j].position;
          float distSq = delta.x * delta.x + delta.y * delta.y;
          float minDist = bodies[i].radius + bodies[j].radius;
          if (distSq < minDist * minDist) {
            float dist = std::sqrt(distSq);
            sf::Vector2f normal = (dist > 0.0f) ? delta / dist : sf::Vector2f(1.0f, 0.0f);
            bodies[i].position += normal * (minDist - dist);
            bodies[i].velocity = -bodies[i].velocity * RESTITUTION;
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
            bodies[i].velocity.x = -bodies[i].velocity.x * RESTITUTION;
          }
        }
      }
    }
    else if (mode == SimulationMode::CCD_60Hz) {
      stepsCount = 1;
      for (size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i].isStatic) continue;

        float timeLeft = DT_60;
        int iter = 0;
        while (timeLeft > 0.0f && iter < 4) {
          float min_tc = timeLeft;
          sf::Vector2f normal(0.0f, 0.0f);
          bool hit = false;

          for (const auto& wall : walls) {
            float tc = 0.0f;
            sf::Vector2f temp_normal(0.0f, 0.0f);
            if (CCD::findCollisionTime(bodies[i], wall, timeLeft, tc, temp_normal)) {
              if (tc < min_tc) {
                min_tc = tc;
                normal = temp_normal;
                hit = true;
              }
            }
          }

          bodies[i].position += bodies[i].velocity * min_tc;
          timeLeft -= min_tc;

          if (hit) {
            CCD::resolveCollision(bodies[i], normal);
          }
          iter++;
        }

        for (size_t j = 0; j < bodies.size(); ++j) {
          if (i == j || !bodies[j].isStatic) continue;
          sf::Vector2f delta = bodies[i].position - bodies[j].position;
          float distSq = delta.x * delta.x + delta.y * delta.y;
          float minDist = bodies[i].radius + bodies[j].radius;
          if (distSq < minDist * minDist) {
            float dist = std::sqrt(distSq);
            sf::Vector2f normal = (dist > 0.0f) ? delta / dist : sf::Vector2f(1.0f, 0.0f);
            bodies[i].position += normal * (minDist - dist);
            bodies[i].velocity -= (1.0f + RESTITUTION) * (bodies[i].velocity.x * normal.x + bodies[i].velocity.y * normal.y) * normal;
          }
        }
      }
    }
    else if (mode == SimulationMode::MultiRate_240Hz) {
      multiRateSystem.update(bodies, walls, DT_60, stepsCount);
    }

    auto end = std::chrono::high_resolution_clock::now();
    simTimeMs = std::chrono::duration<double, std::milli>(end - start).count();

    if (logClock.getElapsedTime().asSeconds() >= 1.0f) {
      logClock.restart();
      std::cout << std::fixed << std::setprecision(2);
      std::cout << "[PHYSICS LOG] Frame Calc Time: " << simTimeMs << " ms | Sub-steps: " << stepsCount << "\n";

      std::cout << "  -> [Yellow Projectile #0]\n"
        << "     Pos: (" << bodies[0].position.x << ", " << bodies[0].position.y << ") px\n"
        << "     Vel: (" << bodies[0].velocity.x << ", " << bodies[0].velocity.y << ") px/s\n"
        << "     Status: " << ((bodies[0].position.x > 515.0f) ? "TUNNELED (FAIL)" : "IN BOUNDS (OK)") << "\n";

      std::cout << "  -> [Purple Tracked Circle #4]\n"
        << "     Pos: (" << bodies[4].position.x << ", " << bodies[4].position.y << ") px\n"
        << "     Vel: (" << bodies[4].velocity.x << ", " << bodies[4].velocity.y << ") px/s\n";
      std::cout << "--------------------------------------------------------\n";
    }
    window.clear(sf::Color(20, 20, 30));
    for (const auto& wall : walls) {
      sf::RectangleShape r(wall.rect.size);
      r.setPosition(wall.rect.position);
      r.setFillColor(wall.color);
      window.draw(r);
    }
    for (const auto& body : bodies) {
      sf::CircleShape c(body.radius);
      c.setOrigin({ body.radius, body.radius });
      c.setPosition(body.position);
      c.setFillColor(body.color);
      window.draw(c);
    }
    std::string modeStr = (mode == SimulationMode::Discrete_60Hz) ? "Discrete Integration (60 Hz)" :
      (mode == SimulationMode::CCD_60Hz) ? "Continuous Collision Detection (60 Hz)" :
      "Multi-Rate Sub-stepping (240 Hz)";
    uiText.setString(
      "SIMULATION CONTROLS:\n"
      "  [Press 1] : Switch to Discrete Mode (60 Hz)\n"
      "  [Press 2] : Switch to Continuous Mode (CCD 60 Hz)\n"
      "  [Press 3] : Switch to Multi-Rate Mode (240 Hz)\n"
      "  [Press R] : Reset Scene & Respawn Projectile\n\n"
      "PHYSICS ENGINE METRICS:\n"
      "  Active Mode           : " + modeStr + "\n" +
      "  Physics Step Time     : " + std::to_string(simTimeMs) + " ms/frame\n" +
      "  Physics Sub-steps     : " + std::to_string(stepsCount) + " steps/frame\n" +
      "  Projectile X Position : " + std::to_string(bodies[0].position.x) + " px\n" +
      "  Projectile Y Position : " + std::to_string(bodies[0].position.y) + " px\n" +
      "  Projectile X Velocity : " + std::to_string(bodies[0].velocity.x) + " px/s\n" +
      "  Red Wall Status       : " + ((bodies[0].position.x > 515.0f) ? "TUNNELED (FAIL)" : "COLLIDED / BOUNCED")
    );
    window.draw(uiText);
    window.display();
}
return 0;
}