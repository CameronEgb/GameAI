#include <SFML/Graphics.hpp>
#include "PositionMatchBehavior.h"
#include "OrientationMatchBehavior.h"
#include "VelocityMatchBehavior.h"
#include "RotationMatchBehavior.h"

int main() {
    sf::RenderWindow window(sf::VideoMode(800, 600), "Steering Behaviors - Part 1");
    window.setFramerateLimit(60);

    Kinematic character{{400, 300}, 0, {0, 0}, 0};
    Kinematic target{{0, 0}, 0, {0, 0}, 0};

    VelocityMatchBehavior velocityMatch;

    sf::CircleShape agent(10);
    agent.setFillColor(sf::Color::Cyan);
    agent.setOrigin(10, 10);

    sf::Clock clock;
    sf::Vector2f prevMouse;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        float dt = clock.restart().asSeconds();
        sf::Vector2f mousePos = (sf::Vector2f)sf::Mouse::getPosition(window);
        target.position = mousePos;

        // Estimate mouse velocity
        target.velocity = (mousePos - prevMouse) / dt;
        prevMouse = mousePos;

        SteeringOutput steering = velocityMatch.calculateSteering(character, target);

        // Integrate motion
        character.velocity += steering.linear * dt;
        character.position += character.velocity * dt;

        agent.setPosition(character.position);

        window.clear(sf::Color::Black);
        window.draw(agent);
        window.display();
    }
}
