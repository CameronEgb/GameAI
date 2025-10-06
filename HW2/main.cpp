// CSC 584/484 Fall 25 Homework 2

#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <memory>
#include <random>
#include <algorithm>

const float PI = 3.14159265f;
const int WINDOW_WIDTH = 1200;
const int WINDOW_HEIGHT = 800;

// Forward declarations
struct Kinematic;
struct SteeringOutput;

// Utility functions
float mapToRange(float rotation) {
    while (rotation > PI) rotation -= 2 * PI;
    while (rotation < -PI) rotation += 2 * PI;
    return rotation;
}

float randomBinomial() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(0.0, 1.0);
    return dis(gen) - dis(gen);
}

float randomFloat(float min, float max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    return dis(gen);
}

// Data structures
struct Kinematic {
    sf::Vector2f position;
    float orientation;
    sf::Vector2f velocity;
    float rotation;
    
    Kinematic() : position(0, 0), orientation(0), velocity(0, 0), rotation(0) {}
};

struct SteeringOutput {
    sf::Vector2f linear;
    float angular;
    
    SteeringOutput() : linear(0, 0), angular(0) {}
};

// Breadcrumb class
class Breadcrumb {
private:
    std::queue<sf::Vector2f> positions;
    int maxCrumbs;
    int dropInterval;
    int counter;
    sf::Color color;
    
public:
    Breadcrumb(int max = 30, int interval = 5, sf::Color c = sf::Color::Blue) 
        : maxCrumbs(max), dropInterval(interval), counter(0), color(c) {}
    
    void update(const sf::Vector2f& pos) {
        counter++;
        if (counter >= dropInterval) {
            counter = 0;
            positions.push(pos);
            if ((int)positions.size() > maxCrumbs) {
                positions.pop();
            }
        }
    }
    
    void draw(sf::RenderWindow& window) {
        std::queue<sf::Vector2f> temp = positions;
        float alpha = 50.0f;
        float alphaInc = 200.0f / maxCrumbs;
        
        while (!temp.empty()) {
            sf::CircleShape crumb(3);
            crumb.setPosition(temp.front() - sf::Vector2f(3, 3));
            sf::Color c = color;
            c.a = std::min(255.0f, alpha);
            crumb.setFillColor(c);
            window.draw(crumb);
            temp.pop();
            alpha += alphaInc;
        }
    }
    
    void clear() {
        while (!positions.empty()) positions.pop();
        counter = 0;
    }
};

// Part 1: Pure virtual SteeringBehavior class
class SteeringBehavior {
public:
    virtual ~SteeringBehavior() {}
    virtual SteeringOutput calculateSteering(const Kinematic& character, const Kinematic& target) = 0;
};

// Position Matching
class PositionMatching : public SteeringBehavior {
private:
    float maxAcceleration;
    
public:
    PositionMatching(float maxAccel = 100.0f) : maxAcceleration(maxAccel) {}
    
    SteeringOutput calculateSteering(const Kinematic& character, const Kinematic& target) override {
        SteeringOutput result;
        sf::Vector2f direction = target.position - character.position;
        float distance = std::sqrt(direction.x * direction.x + direction.y * direction.y);
        
        if (distance > 0.001f) {
            direction /= distance;
            result.linear = direction * maxAcceleration;
        }
        return result;
    }
};

// Velocity Matching
class VelocityMatching : public SteeringBehavior {
private:
    float maxAcceleration;
    float timeToTarget;
    
public:
    VelocityMatching(float maxAccel = 100.0f, float time = 0.1f) 
        : maxAcceleration(maxAccel), timeToTarget(time) {}
    
    SteeringOutput calculateSteering(const Kinematic& character, const Kinematic& target) override {
        SteeringOutput result;
        result.linear = (target.velocity - character.velocity) / timeToTarget;
        
        float length = std::sqrt(result.linear.x * result.linear.x + result.linear.y * result.linear.y);
        if (length > maxAcceleration) {
            result.linear = (result.linear / length) * maxAcceleration;
        }
        return result;
    }
};

// Orientation Matching
class OrientationMatching : public SteeringBehavior {
private:
    float maxAngularAcceleration;
    float maxRotation;
    float targetRadius;
    float slowRadius;
    float timeToTarget;
    
public:
    OrientationMatching(float maxAngAccel = 5.0f, float maxRot = 2.0f) 
        : maxAngularAcceleration(maxAngAccel), maxRotation(maxRot),
          targetRadius(0.01f), slowRadius(0.5f), timeToTarget(0.1f) {}
    
    SteeringOutput calculateSteering(const Kinematic& character, const Kinematic& target) override {
        SteeringOutput result;
        float rotation = target.orientation - character.orientation;
        rotation = mapToRange(rotation);
        float rotationSize = std::abs(rotation);
        
        if (rotationSize < targetRadius) return result;
        
        float targetRotation;
        if (rotationSize > slowRadius) {
            targetRotation = maxRotation;
        } else {
            targetRotation = maxRotation * rotationSize / slowRadius;
        }
        
        targetRotation *= rotation / rotationSize;
        result.angular = (targetRotation - character.rotation) / timeToTarget;
        
        float angularAcceleration = std::abs(result.angular);
        if (angularAcceleration > maxAngularAcceleration) {
            result.angular /= angularAcceleration;
            result.angular *= maxAngularAcceleration;
        }
        
        return result;
    }
};

// Rotation Matching
class RotationMatching : public SteeringBehavior {
private:
    float maxAngularAcceleration;
    float timeToTarget;
    
public:
    RotationMatching(float maxAngAccel = 5.0f, float time = 0.1f)
        : maxAngularAcceleration(maxAngAccel), timeToTarget(time) {}
    
    SteeringOutput calculateSteering(const Kinematic& character, const Kinematic& target) override {
        SteeringOutput result;
        result.angular = (target.rotation - character.rotation) / timeToTarget;
        
        if (std::abs(result.angular) > maxAngularAcceleration) {
            result.angular = (result.angular / std::abs(result.angular)) * maxAngularAcceleration;
        }
        return result;
    }
};

// Part 2: Arrive behavior
class Arrive : public SteeringBehavior {
private:
    float maxAcceleration;
    float maxSpeed;
    float targetRadius;
    float slowRadius;
    float timeToTarget;
    
public:
    Arrive(float maxAccel = 200.0f, float maxSpd = 100.0f, float targetRad = 5.0f, 
           float slowRad = 100.0f, float time = 0.1f)
        : maxAcceleration(maxAccel), maxSpeed(maxSpd), targetRadius(targetRad),
          slowRadius(slowRad), timeToTarget(time) {}
    
    SteeringOutput calculateSteering(const Kinematic& character, const Kinematic& target) override {
        SteeringOutput result;
        sf::Vector2f direction = target.position - character.position;
        float distance = std::sqrt(direction.x * direction.x + direction.y * direction.y);
        
        if (distance < targetRadius) return result;
        
        float targetSpeed;
        if (distance > slowRadius) {
            targetSpeed = maxSpeed;
        } else {
            targetSpeed = maxSpeed * distance / slowRadius;
        }
        
        sf::Vector2f targetVelocity = direction;
        targetVelocity = (targetVelocity / distance) * targetSpeed;
        
        result.linear = targetVelocity - character.velocity;
        result.linear /= timeToTarget;
        
        float length = std::sqrt(result.linear.x * result.linear.x + result.linear.y * result.linear.y);
        if (length > maxAcceleration) {
            result.linear = (result.linear / length) * maxAcceleration;
        }
        
        return result;
    }
};

// Align behavior
class Align : public SteeringBehavior {
private:
    float maxAngularAcceleration;
    float maxRotation;
    float targetRadius;
    float slowRadius;
    float timeToTarget;
    
public:
    Align(float maxAngAccel = 5.0f, float maxRot = 2.0f, float targetRad = 0.01f,
          float slowRad = 0.5f, float time = 0.1f)
        : maxAngularAcceleration(maxAngAccel), maxRotation(maxRot),
          targetRadius(targetRad), slowRadius(slowRad), timeToTarget(time) {}
    
    SteeringOutput calculateSteering(const Kinematic& character, const Kinematic& target) override {
        SteeringOutput result;
        float rotation = target.orientation - character.orientation;
        rotation = mapToRange(rotation);
        float rotationSize = std::abs(rotation);
        
        if (rotationSize < targetRadius) return result;
        
        float targetRotation;
        if (rotationSize > slowRadius) {
            targetRotation = maxRotation;
        } else {
            targetRotation = maxRotation * rotationSize / slowRadius;
        }
        
        targetRotation *= rotation / rotationSize;
        result.angular = (targetRotation - character.rotation) / timeToTarget;
        
        float angularAcceleration = std::abs(result.angular);
        if (angularAcceleration > maxAngularAcceleration) {
            result.angular /= angularAcceleration;
            result.angular *= maxAngularAcceleration;
        }
        
        return result;
    }
};

// Face behavior (align to direction of movement)
class Face : public SteeringBehavior {
private:
    Align align;
    
public:
    Face() : align() {}
    
    SteeringOutput calculateSteering(const Kinematic& character, const Kinematic& target) override {
        sf::Vector2f direction = target.position - character.position;
        
        if (direction.x == 0 && direction.y == 0) {
            return SteeringOutput();
        }
        
        Kinematic faceTarget;
        faceTarget.orientation = std::atan2(direction.y, direction.x);
        
        return align.calculateSteering(character, faceTarget);
    }
};

// Look Where You're Going
class LookWhereYoureGoing : public SteeringBehavior {
private:
    Align align;
    
public:
    LookWhereYoureGoing() : align() {}
    
    SteeringOutput calculateSteering(const Kinematic& character, const Kinematic& target) override {
        if (character.velocity.x == 0 && character.velocity.y == 0) {
            return SteeringOutput();
        }
        
        Kinematic alignTarget;
        alignTarget.orientation = std::atan2(character.velocity.y, character.velocity.x);
        
        return align.calculateSteering(character, alignTarget);
    }
};

// Part 3: Wander behavior
class Wander : public SteeringBehavior {
private:
    float wanderOffset;
    float wanderRadius;
    float wanderRate;
    float wanderOrientation;
    float maxAcceleration;
    LookWhereYoureGoing lookWhereGoing;
    
public:
    Wander(float offset = 60.0f, float radius = 40.0f, float rate = 0.5f, float maxAccel = 80.0f)
        : wanderOffset(offset), wanderRadius(radius), wanderRate(rate),
          maxAcceleration(maxAccel), wanderOrientation(0), lookWhereGoing() {}
    
    SteeringOutput calculateSteering(const Kinematic& character, const Kinematic& target) override {
        wanderOrientation += randomBinomial() * wanderRate;
        
        float targetOrientation = wanderOrientation + character.orientation;
        
        sf::Vector2f characterOrientationVec(std::cos(character.orientation), 
                                            std::sin(character.orientation));
        sf::Vector2f wanderCircleCenter = character.position + characterOrientationVec * wanderOffset;
        
        sf::Vector2f wanderTarget;
        wanderTarget.x = wanderCircleCenter.x + wanderRadius * std::cos(targetOrientation);
        wanderTarget.y = wanderCircleCenter.y + wanderRadius * std::sin(targetOrientation);
        
        SteeringOutput result;
        result.linear = wanderTarget - character.position;
        float length = std::sqrt(result.linear.x * result.linear.x + 
                                result.linear.y * result.linear.y);
        if (length > 0) {
            result.linear = (result.linear / length) * maxAcceleration;
        }
        
        SteeringOutput lookSteering = lookWhereGoing.calculateSteering(character, target);
        result.angular = lookSteering.angular;
        
        return result;
    }
};

// Part 4: Flocking behaviors
class Separation : public SteeringBehavior {
private:
    float threshold;
    float decayCoefficient;
    float maxAcceleration;
    std::vector<Kinematic*>* boids;
    
public:
    Separation(std::vector<Kinematic*>* b, float thresh = 50.0f, float decay = 5000.0f, 
               float maxAccel = 200.0f)
        : boids(b), threshold(thresh), decayCoefficient(decay), maxAcceleration(maxAccel) {}
    
    SteeringOutput calculateSteering(const Kinematic& character, const Kinematic& target) override {
        SteeringOutput result;
        
        for (auto& boid : *boids) {
            if (boid->position == character.position) continue;
            
            sf::Vector2f direction = character.position - boid->position;
            float distance = std::sqrt(direction.x * direction.x + direction.y * direction.y);
            
            if (distance < threshold && distance > 0) {
                float strength = std::min(decayCoefficient / (distance * distance), maxAcceleration);
                direction = (direction / distance) * strength;
                result.linear += direction;
            }
        }
        
        float length = std::sqrt(result.linear.x * result.linear.x + result.linear.y * result.linear.y);
        if (length > maxAcceleration) {
            result.linear = (result.linear / length) * maxAcceleration;
        }
        
        return result;
    }
};

class Cohesion : public SteeringBehavior {
private:
    float neighborhoodRadius;
    float maxAcceleration;
    std::vector<Kinematic*>* boids;
    Arrive arrive;
    
public:
    Cohesion(std::vector<Kinematic*>* b, float radius = 150.0f, float maxAccel = 100.0f)
        : boids(b), neighborhoodRadius(radius), maxAcceleration(maxAccel),
          arrive(maxAccel, 100.0f, 10.0f, 50.0f) {}
    
    SteeringOutput calculateSteering(const Kinematic& character, const Kinematic& target) override {
        sf::Vector2f centerOfMass(0, 0);
        int count = 0;
        
        for (auto& boid : *boids) {
            if (boid->position == character.position) continue;
            
            sf::Vector2f direction = boid->position - character.position;
            float distance = std::sqrt(direction.x * direction.x + direction.y * direction.y);
            
            if (distance < neighborhoodRadius) {
                centerOfMass += boid->position;
                count++;
            }
        }
        
        if (count == 0) return SteeringOutput();
        
        centerOfMass /= static_cast<float>(count);
        
        Kinematic cohesionTarget;
        cohesionTarget.position = centerOfMass;
        
        return arrive.calculateSteering(character, cohesionTarget);
    }
};

class Alignment : public SteeringBehavior {
private:
    float neighborhoodRadius;
    float maxAcceleration;
    std::vector<Kinematic*>* boids;
    VelocityMatching velocityMatch;
    
public:
    Alignment(std::vector<Kinematic*>* b, float radius = 100.0f, float maxAccel = 50.0f)
        : boids(b), neighborhoodRadius(radius), maxAcceleration(maxAccel),
          velocityMatch(maxAccel) {}
    
    SteeringOutput calculateSteering(const Kinematic& character, const Kinematic& target) override {
        sf::Vector2f averageVelocity(0, 0);
        int count = 0;
        
        for (auto& boid : *boids) {
            if (boid->position == character.position) continue;
            
            sf::Vector2f direction = boid->position - character.position;
            float distance = std::sqrt(direction.x * direction.x + direction.y * direction.y);
            
            if (distance < neighborhoodRadius) {
                averageVelocity += boid->velocity;
                count++;
            }
        }
        
        if (count == 0) return SteeringOutput();
        
        averageVelocity /= static_cast<float>(count);
        
        Kinematic alignTarget;
        alignTarget.velocity = averageVelocity;
        
        return velocityMatch.calculateSteering(character, alignTarget);
    }
};

// Blended Steering
class BlendedSteering : public SteeringBehavior {
private:
    struct BehaviorAndWeight {
        SteeringBehavior* behavior;
        float weight;
    };
    
    std::vector<BehaviorAndWeight> behaviors;
    float maxAcceleration;
    float maxAngular;
    
public:
    BlendedSteering(float maxAccel = 200.0f, float maxAng = 5.0f)
        : maxAcceleration(maxAccel), maxAngular(maxAng) {}
    
    void addBehavior(SteeringBehavior* behavior, float weight) {
        behaviors.push_back({behavior, weight});
    }
    
    SteeringOutput calculateSteering(const Kinematic& character, const Kinematic& target) override {
        SteeringOutput result;
        
        for (auto& bw : behaviors) {
            SteeringOutput steering = bw.behavior->calculateSteering(character, target);
            result.linear += steering.linear * bw.weight;
            result.angular += steering.angular * bw.weight;
        }
        
        float length = std::sqrt(result.linear.x * result.linear.x + result.linear.y * result.linear.y);
        if (length > maxAcceleration) {
            result.linear = (result.linear / length) * maxAcceleration;
        }
        
        if (std::abs(result.angular) > maxAngular) {
            result.angular = (result.angular / std::abs(result.angular)) * maxAngular;
        }
        
        return result;
    }
};

// Character class
class Character {
private:
    Kinematic kinematic;
    sf::ConvexShape shape;
    Breadcrumb breadcrumbs;
    SteeringBehavior* currentBehavior;
    float maxSpeed;
    float maxRotation;
    
public:
    Character(sf::Vector2f startPos = sf::Vector2f(400, 300), sf::Color color = sf::Color::Red,
              int breadcrumbMax = 30, int breadcrumbInterval = 5)
        : breadcrumbs(breadcrumbMax, breadcrumbInterval, color), 
          currentBehavior(nullptr), maxSpeed(150.0f), maxRotation(3.0f) {
        
        kinematic.position = startPos;
        kinematic.orientation = 0;
        
        // Create triangle shape
        shape.setPointCount(3);
        shape.setPoint(0, sf::Vector2f(15, 0));
        shape.setPoint(1, sf::Vector2f(-10, -8));
        shape.setPoint(2, sf::Vector2f(-10, 8));
        shape.setFillColor(color);
        shape.setOrigin(0, 0);
    }
    
    void setBehavior(SteeringBehavior* behavior) {
        currentBehavior = behavior;
    }
    
    void update(float deltaTime, const Kinematic& target) {
        if (!currentBehavior) return;
        
        SteeringOutput steering = currentBehavior->calculateSteering(kinematic, target);
        
        // Update position and velocity
        kinematic.position += kinematic.velocity * deltaTime;
        kinematic.orientation += kinematic.rotation * deltaTime;
        
        kinematic.velocity += steering.linear * deltaTime;
        kinematic.rotation += steering.angular * deltaTime;
        
        // Cap velocity
        float speed = std::sqrt(kinematic.velocity.x * kinematic.velocity.x + 
                               kinematic.velocity.y * kinematic.velocity.y);
        if (speed > maxSpeed) {
            kinematic.velocity = (kinematic.velocity / speed) * maxSpeed;
        }
        
        // Cap rotation
        if (std::abs(kinematic.rotation) > maxRotation) {
            kinematic.rotation = (kinematic.rotation / std::abs(kinematic.rotation)) * maxRotation;
        }
        
        // Handle boundaries (wrap around)
        if (kinematic.position.x < 0) kinematic.position.x = WINDOW_WIDTH;
        if (kinematic.position.x > WINDOW_WIDTH) kinematic.position.x = 0;
        if (kinematic.position.y < 0) kinematic.position.y = WINDOW_HEIGHT;
        if (kinematic.position.y > WINDOW_HEIGHT) kinematic.position.y = 0;
        
        // Update breadcrumbs
        breadcrumbs.update(kinematic.position);
        
        // Update shape
        shape.setPosition(kinematic.position);
        shape.setRotation(kinematic.orientation * 180 / PI);
    }
    
    void draw(sf::RenderWindow& window) {
        breadcrumbs.draw(window);
        window.draw(shape);
    }
    
    Kinematic& getKinematic() { return kinematic; }
    
    void clearBreadcrumbs() { breadcrumbs.clear(); }
    
    void setMaxSpeed(float speed) { maxSpeed = speed; }
    void setPosition(sf::Vector2f pos) { 
        kinematic.position = pos;
        shape.setPosition(pos);
    }
};

// Boid for flocking
class Boid {
public:
    Kinematic kinematic;
    sf::CircleShape shape;
    Breadcrumb breadcrumbs;
    BlendedSteering* flockingBehavior;
    float maxSpeed;
    
    Boid(sf::Vector2f startPos, sf::Color color = sf::Color::Blue)
        : breadcrumbs(20, 10, color), maxSpeed(120.0f) {
        
        kinematic.position = startPos;
        kinematic.velocity = sf::Vector2f(randomFloat(-50, 50), randomFloat(-50, 50));
        kinematic.orientation = std::atan2(kinematic.velocity.y, kinematic.velocity.x);
        
        shape.setRadius(5);
        shape.setFillColor(color);
        shape.setOrigin(5, 5);
        
        flockingBehavior = nullptr;
    }
    
    void update(float deltaTime) {
        if (!flockingBehavior) return;
        
        SteeringOutput steering = flockingBehavior->calculateSteering(kinematic, kinematic);
        
        kinematic.position += kinematic.velocity * deltaTime;
        kinematic.velocity += steering.linear * deltaTime;
        
        float speed = std::sqrt(kinematic.velocity.x * kinematic.velocity.x + 
                               kinematic.velocity.y * kinematic.velocity.y);
        if (speed > maxSpeed) {
            kinematic.velocity = (kinematic.velocity / speed) * maxSpeed;
        }
        
        if (speed > 0.01f) {
            kinematic.orientation = std::atan2(kinematic.velocity.y, kinematic.velocity.x);
        }
        
        // Wrap around boundaries
        if (kinematic.position.x < 0) kinematic.position.x = WINDOW_WIDTH;
        if (kinematic.position.x > WINDOW_WIDTH) kinematic.position.x = 0;
        if (kinematic.position.y < 0) kinematic.position.y = WINDOW_HEIGHT;
        if (kinematic.position.y > WINDOW_HEIGHT) kinematic.position.y = 0;
        
        breadcrumbs.update(kinematic.position);
        shape.setPosition(kinematic.position);
    }
    
    void draw(sf::RenderWindow& window) {
        breadcrumbs.draw(window);
        
        // Draw direction indicator
        sf::RectangleShape direction(sf::Vector2f(15, 2));
        direction.setPosition(kinematic.position);
        direction.setRotation(kinematic.orientation * 180 / PI);
        direction.setFillColor(sf::Color::Red);
        window.draw(direction);
        
        window.draw(shape);
    }
};

// Main application
int main() {
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT),
                            "Steering Behaviors Demo (1: VelMatch, 2: Arrive+Align, 3: Wander, 4: Boids)");
    window.setFramerateLimit(60);

    sf::Clock clock;
    int currentMode = 1;

    // Common behaviors
    VelocityMatching velocityMatch;
    Arrive arriveBehavior;
    Align alignBehavior;
    Wander wanderSmooth(60.0f, 40.0f, 0.5f);
    Wander wanderErratic(80.0f, 60.0f, 1.0f);

    // Characters for cases 1–3
    Character follower(sf::Vector2f(600, 400), sf::Color::Red);
    Character leader(sf::Vector2f(300, 400), sf::Color::Green);

    // Characters for arrive/align case
    Character arriveChar(sf::Vector2f(600, 400), sf::Color::Cyan);
    Character alignChar(sf::Vector2f(600, 400), sf::Color::Yellow);

    // Wander sets (two groups of 5)
    std::vector<std::unique_ptr<Character>> wanderSet1;
    std::vector<std::unique_ptr<Character>> wanderSet2;
    for (int i = 0; i < 5; ++i) {
        wanderSet1.push_back(std::make_unique<Character>(
            sf::Vector2f(randomFloat(100, 1100), randomFloat(100, 700)), sf::Color::Blue));
        wanderSet2.push_back(std::make_unique<Character>(
            sf::Vector2f(randomFloat(100, 1100), randomFloat(100, 700)), sf::Color::Magenta));
    }

    // Boids
    std::vector<std::unique_ptr<Boid>> flock;
    std::vector<Kinematic*> flockKinematics;
    auto initFlocking = [&]() {
        flock.clear();
        flockKinematics.clear();
        int numBoids = 25;
        for (int i = 0; i < numBoids; ++i) {
            sf::Vector2f pos(randomFloat(100, WINDOW_WIDTH - 100),
                             randomFloat(100, WINDOW_HEIGHT - 100));
            sf::Color color;
            if (i % 3 == 0) color = sf::Color::Cyan;
            else if (i % 3 == 1) color = sf::Color::Magenta;
            else color = sf::Color::Yellow;
            flock.push_back(std::make_unique<Boid>(pos, color));
            flockKinematics.push_back(&flock.back()->kinematic);
        }
        for (auto& boid : flock) {
            auto sep = new Separation(&flockKinematics, 40.0f, 5000.0f, 250.0f);
            auto coh = new Cohesion(&flockKinematics, 100.0f, 80.0f);
            auto ali = new Alignment(&flockKinematics, 80.0f, 100.0f);
            boid->flockingBehavior = new BlendedSteering();
            boid->flockingBehavior->addBehavior(sep, 2.0f);
            boid->flockingBehavior->addBehavior(coh, 0.8f);
            boid->flockingBehavior->addBehavior(ali, 1.0f);
        }
    };
    initFlocking();

    // Font/text
    sf::Font font;
    font.loadFromFile("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf");
    sf::Text modeText("Mode 1: Velocity Matching", font, 20);
    modeText.setPosition(10, 10);

    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Escape) window.close();
                if (event.key.code == sf::Keyboard::Num1) currentMode = 1;
                if (event.key.code == sf::Keyboard::Num2) currentMode = 2;
                if (event.key.code == sf::Keyboard::Num3) currentMode = 3;
                if (event.key.code == sf::Keyboard::Num4) {
                    currentMode = 4;
                    initFlocking();
                }
            }
        }

        window.clear(sf::Color(30, 30, 40));

        switch (currentMode) {
            case 1: {
                modeText.setString("Case 1: Velocity Matching");
                // Leader moves in a circle
                float t = clock.getElapsedTime().asSeconds();
                leader.getKinematic().position = {600 + 200 * std::cos(t), 400 + 200 * std::sin(t)};
                leader.getKinematic().velocity = {-200 * std::sin(t), 200 * std::cos(t)};
                follower.setBehavior(&velocityMatch);
                follower.update(dt, leader.getKinematic());
                leader.draw(window);
                follower.draw(window);
                break;
            }
            case 2: {
                modeText.setString("Case 2: Arrive and Align");
                Kinematic target;
                target.position = sf::Vector2f(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
                target.orientation = std::sin(clock.getElapsedTime().asSeconds()) * PI;

                arriveChar.setBehavior(&arriveBehavior);
                alignChar.setBehavior(&alignBehavior);

                arriveChar.update(dt, target);
                alignChar.update(dt, target);

                arriveChar.draw(window);
                alignChar.draw(window);
                break;
            }
            case 3: {
                modeText.setString("Case 3: Wander (Two Groups)");
                for (auto& c : wanderSet1) {
                    c->setBehavior(&wanderSmooth);
                    c->update(dt, c->getKinematic());
                    c->draw(window);
                }
                for (auto& c : wanderSet2) {
                    c->setBehavior(&wanderErratic);
                    c->update(dt, c->getKinematic());
                    c->draw(window);
                }
                break;
            }
            case 4: {
                modeText.setString("Case 4: Reynolds Boids");
                for (auto& b : flock) {
                    b->update(dt);
                    b->draw(window);
                }
                break;
            }
        }

        window.draw(modeText);
        window.display();
    }
    return 0;
}
