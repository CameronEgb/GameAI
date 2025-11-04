#include "steering.h"

// Utilities
float mapToRange(float rotation) {
    while (rotation > PI) rotation -= 2.f * PI;
    while (rotation < -PI) rotation += 2.f * PI;
    return rotation;
}

float randomBinomial() {
    static std::mt19937 gen(std::random_device{}());
    static std::uniform_real_distribution<float> dis(0.f, 1.f);
    return dis(gen) - dis(gen);
}

float randomFloat(float a, float b) {
    static std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<float> dis(a, b);
    return dis(gen);
}

// Breadcrumb
Breadcrumb::Breadcrumb(int maxCrumbs_, int dropInterval_, sf::Color c)
    : maxCrumbs(maxCrumbs_), dropInterval(dropInterval_), counter(0), color(c) {}

void Breadcrumb::update(const sf::Vector2f &pos) {
    if (++counter >= dropInterval) {
        counter = 0;
        q.push(pos);
        if ((int)q.size() > maxCrumbs) q.pop();
    }
}

void Breadcrumb::draw(sf::RenderWindow &win) {
    std::queue<sf::Vector2f> temp = q;
    float alpha = 50.f;
    float inc = 200.f / std::max(1, maxCrumbs);
    while (!temp.empty()) {
        sf::CircleShape c(3.f);
        c.setPosition(temp.front() - sf::Vector2f(3.f,3.f));
        sf::Color col = color;
        col.a = static_cast<std::uint8_t>(std::min(255.f, alpha));
        c.setFillColor(col);
        win.draw(c);
        temp.pop();
        alpha += inc;
    }
}

void Breadcrumb::clear() {
    while (!q.empty()) q.pop();
    counter = 0;
}

// PositionMatching
PositionMatching::PositionMatching(float m) : maxAccel(m) {}
SteeringOutput PositionMatching::calculateSteering(const Kinematic &c, const Kinematic &t) {
    SteeringOutput out;
    sf::Vector2f dir = t.position - c.position;
    float d = std::sqrt(dir.x*dir.x + dir.y*dir.y);
    if (d > 0.001f) {
        dir /= d;
        out.linear = dir * maxAccel;
    }
    return out;
}

// VelocityMatching
VelocityMatching::VelocityMatching(float maxA, float time) : maxAccel(maxA), timeToTarget(time) {}
SteeringOutput VelocityMatching::calculateSteering(const Kinematic &c, const Kinematic &t) {
    SteeringOutput out;
    out.linear = (t.velocity - c.velocity) / timeToTarget;
    float mag = std::sqrt(out.linear.x*out.linear.x + out.linear.y*out.linear.y);
    if (mag > maxAccel) out.linear = (out.linear / mag) * maxAccel;
    return out;
}

FastVelocityMatching::FastVelocityMatching() : VelocityMatching(400.f, 0.05f) {}

// OrientationMatching
OrientationMatching::OrientationMatching(float maxAA, float maxR)
    : maxAngAccel(maxAA), maxRotation(maxR), targetRadius(0.01f), slowRadius(0.5f), timeToTarget(0.1f) {}
SteeringOutput OrientationMatching::calculateSteering(const Kinematic &c, const Kinematic &t) {
    SteeringOutput out;
    float rotation = t.orientation - c.orientation;
    rotation = mapToRange(rotation);
    float rotSize = std::abs(rotation);
    if (rotSize < targetRadius) return out;
    float targetRot = (rotSize > slowRadius) ? maxRotation : maxRotation * rotSize / slowRadius;
    targetRot *= rotation / rotSize;
    out.angular = (targetRot - c.rotation) / timeToTarget;
    float angularAccel = std::abs(out.angular);
    if (angularAccel > maxAngAccel) {
        out.angular = (out.angular / angularAccel) * maxAngAccel;
    }
    return out;
}

// RotationMatching
RotationMatching::RotationMatching(float maxA, float time) : maxAngAccel(maxA), timeToTarget(time) {}
SteeringOutput RotationMatching::calculateSteering(const Kinematic &c, const Kinematic &t) {
    SteeringOutput out;
    out.angular = (t.rotation - c.rotation) / timeToTarget;
    if (std::abs(out.angular) > maxAngAccel)
        out.angular = (out.angular / std::abs(out.angular)) * maxAngAccel;
    return out;
}

// Arrive
Arrive::Arrive(float maxAccel,float maxSpd,float tRad,float sRad,float time)
    : maxAcceleration(maxAccel), maxSpeed(maxSpd), targetRadius(tRad), slowRadius(sRad), timeToTarget(time) {}
SteeringOutput Arrive::calculateSteering(const Kinematic &c, const Kinematic &t) {
    SteeringOutput out;
    sf::Vector2f direction = t.position - c.position;
    float dist = std::sqrt(direction.x*direction.x + direction.y*direction.y);
    if (dist < targetRadius) return out;
    float targetSpeed = (dist > slowRadius) ? maxSpeed : maxSpeed * dist / slowRadius;
    sf::Vector2f targetVel(0.f, 0.f);
    if (dist > 0.0001f) targetVel = (direction / dist) * targetSpeed;
    out.linear = (targetVel - c.velocity) / timeToTarget;
    float mag = std::sqrt(out.linear.x*out.linear.x + out.linear.y*out.linear.y);
    if (mag > maxAcceleration) out.linear = (out.linear / mag) * maxAcceleration;
    return out;
}

// Align
Align::Align(float maxAngAccel, float maxRot, float tRad, float sRad, float time)
    : maxAngularAcceleration(maxAngAccel), maxRotation(maxRot), targetRadius(tRad), slowRadius(sRad), timeToTarget(time) {}
SteeringOutput Align::calculateSteering(const Kinematic &c, const Kinematic &t) {
    SteeringOutput out;
    float rotation = t.orientation - c.orientation;
    rotation = mapToRange(rotation);
    float rotationSize = std::abs(rotation);
    if (rotationSize < targetRadius) return out;
    float targetRotation = (rotationSize > slowRadius) ? maxRotation : maxRotation * rotationSize / slowRadius;
    targetRotation *= rotation / rotationSize;
    out.angular = (targetRotation - c.rotation) / timeToTarget;
    float angularAccel = std::abs(out.angular);
    if (angularAccel > maxAngularAcceleration) {
        out.angular = (out.angular / angularAccel) * maxAngularAcceleration;
    }
    return out;
}

SmoothAlign::SmoothAlign() : Align(2.5f, 1.5f, 0.01f, 0.6f, 0.1f) {}

// Face
Face::Face() : align() {}
SteeringOutput Face::calculateSteering(const Kinematic &c, const Kinematic &t) {
    sf::Vector2f dir = t.position - c.position;
    if (dir.x == 0.f && dir.y == 0.f) return {};
    Kinematic target;
    target.orientation = std::atan2(dir.y, dir.x);
    return align.calculateSteering(c, target);
}

// ArriveAndAlign
ArriveAndAlign::ArriveAndAlign(float arrive_maxAccel,
                               float arrive_maxSpeed,
                               float arrive_targetRadius,
                               float arrive_slowRadius,
                               float arrive_timeToTarget,
                               float align_maxAngularAccel,
                               float align_maxRotation,
                               float align_targetRadius,
                               float align_slowRadius,
                               float align_timeToTarget)
    : arrive(arrive_maxAccel, arrive_maxSpeed, arrive_targetRadius, arrive_slowRadius, arrive_timeToTarget),
      align(align_maxAngularAccel, align_maxRotation, align_targetRadius, align_slowRadius, align_timeToTarget) {}

SteeringOutput ArriveAndAlign::calculateSteering(const Kinematic &character, const Kinematic &target) {
    SteeringOutput sArrive = arrive.calculateSteering(character, target);
    SteeringOutput sAlign  = align.calculateSteering(character, target);
    SteeringOutput out;
    out.linear  = sArrive.linear;
    out.angular = sAlign.angular;
    return out;
}

// LookWhereYoureGoing
LookWhereYoureGoing::LookWhereYoureGoing() : align() {}
SteeringOutput LookWhereYoureGoing::calculateSteering(const Kinematic &c, const Kinematic & /*t*/) {
    if (c.velocity.x == 0.f && c.velocity.y == 0.f) return {};
    Kinematic target;
    target.orientation = std::atan2(c.velocity.y, c.velocity.x);
    return align.calculateSteering(c, target);
}

// WallAvoidance
WallAvoidance::WallAvoidance(float margin, float maxAcc, float detectDist)
    : wallMargin(margin), maxAcceleration(maxAcc), detectionDistance(detectDist) {}
SteeringOutput WallAvoidance::calculateSteering(const Kinematic &c, const Kinematic & /*t*/) {
    SteeringOutput out;
    sf::Vector2f avoid(0.f, 0.f);
    float dl = c.position.x;
    float dr = WINDOW_WIDTH - c.position.x;
    float dt = c.position.y;
    float db = WINDOW_HEIGHT - c.position.y;
    if (dl < detectionDistance) avoid.x += (detectionDistance - dl)/detectionDistance * maxAcceleration;
    if (dr < detectionDistance) avoid.x -= (detectionDistance - dr)/detectionDistance * maxAcceleration;
    if (dt < detectionDistance) avoid.y += (detectionDistance - dt)/detectionDistance * maxAcceleration;
    if (db < detectionDistance) avoid.y -= (detectionDistance - db)/detectionDistance * maxAcceleration;
    out.linear = avoid;
    return out;
}

// Wander
Wander::Wander(float offset, float radius, float rate, float maxA)
    : wanderOffset(offset), wanderRadius(radius), wanderRate(rate), wanderOrientation(0.f), maxAcceleration(maxA) {}
SteeringOutput Wander::calculateSteering(const Kinematic &c, const Kinematic &t) {
    wanderOrientation += randomBinomial() * wanderRate;
    float targetOrientation = wanderOrientation + c.orientation;
    sf::Vector2f orientationVec(std::cos(c.orientation), std::sin(c.orientation));
    sf::Vector2f circleCenter = c.position + orientationVec * wanderOffset;
    sf::Vector2f wanderTarget(circleCenter.x + wanderRadius * std::cos(targetOrientation),
                              circleCenter.y + wanderRadius * std::sin(targetOrientation));
    SteeringOutput out;
    out.linear = wanderTarget - c.position;
    float mag = std::sqrt(out.linear.x*out.linear.x + out.linear.y*out.linear.y);
    if (mag > 0.f) out.linear = (out.linear / mag) * maxAcceleration;
    SteeringOutput look = lwg.calculateSteering(c, t);
    out.angular = look.angular;
    return out;
}

// WanderKinematic
WanderKinematic::WanderKinematic(float offset, float radius, float rate, float maxA, float maxR)
    : wanderOffset(offset), wanderRadius(radius), wanderRate(rate), wanderOrientation(0.f), maxAcceleration(maxA), maxRotation(maxR) {}
SteeringOutput WanderKinematic::calculateSteering(const Kinematic &c, const Kinematic & /*t*/) {
    wanderOrientation += randomBinomial() * wanderRate;
    float targetOrientation = wanderOrientation + c.orientation;
    sf::Vector2f orientationVec(std::cos(c.orientation), std::sin(c.orientation));
    sf::Vector2f circleCenter = c.position + orientationVec * wanderOffset;
    sf::Vector2f wanderTarget(circleCenter.x + wanderRadius * std::cos(targetOrientation),
                              circleCenter.y + wanderRadius * std::sin(targetOrientation));
    SteeringOutput out;
    out.linear = wanderTarget - c.position;
    float mag = std::sqrt(out.linear.x*out.linear.x + out.linear.y*out.linear.y);
    if (mag > 0.f) out.linear = (out.linear / mag) * maxAcceleration;
    sf::Vector2f toTarget = wanderTarget - c.position;
    float desiredOrientation = std::atan2(toTarget.y, toTarget.x);
    float rotation = mapToRange(desiredOrientation - c.orientation);
    out.angular = rotation * 3.f;
    if (std::abs(out.angular) > maxRotation) out.angular = (out.angular / std::abs(out.angular)) * maxRotation;
    return out;
}

// Separation
Separation::Separation(std::vector<Kinematic*> *b, float thresh, float decay, float maxA)
    : threshold(thresh), decayCoefficient(decay), maxAcceleration(maxA), boids(b) {}
SteeringOutput Separation::calculateSteering(const Kinematic &c, const Kinematic & /*t*/) {
    SteeringOutput out;
    for (auto b : *boids) {
        if (b->position == c.position) continue;
        sf::Vector2f direction = c.position - b->position;
        float dist = std::sqrt(direction.x*direction.x + direction.y*direction.y);
        if (dist < threshold && dist > 0.f) {
            float strength = std::min(decayCoefficient / (dist*dist), maxAcceleration);
            direction = (direction / dist) * strength;
            out.linear += direction;
        }
    }
    float mag = std::sqrt(out.linear.x*out.linear.x + out.linear.y*out.linear.y);
    if (mag > maxAcceleration) out.linear = (out.linear / mag) * maxAcceleration;
    return out;
}

// Cohesion
Cohesion::Cohesion(std::vector<Kinematic*> *b, float radius, float maxA)
    : neighborhoodRadius(radius), maxAcceleration(maxA), boids(b), arrive(maxA, 100.f, 10.f, 50.f) {}
SteeringOutput Cohesion::calculateSteering(const Kinematic &c, const Kinematic & /*t*/) {
    sf::Vector2f center(0.f, 0.f);
    int count = 0;
    for (auto b : *boids) {
        if (b->position == c.position) continue;
        sf::Vector2f dir = b->position - c.position;
        float dist = std::sqrt(dir.x*dir.x + dir.y*dir.y);
        if (dist < neighborhoodRadius) {
            center += b->position;
            count++;
        }
    }
    if (count == 0) return {};
    center /= static_cast<float>(count);
    Kinematic target;
    target.position = center;
    return arrive.calculateSteering(c, target);
}

// Character
Character::Character(sf::Vector2f start, sf::Color color)
    : breadcrumbs(20, 10, color), currentBehavior(nullptr), maxSpeed(200.f), maxRotation(5.f) { // Assumed maxRotation
    kinematic.position = start;
    kinematic.velocity = sf::Vector2f(randomFloat(-50.f, 50.f), randomFloat(-50.f, 50.f));
    kinematic.orientation = std::atan2(kinematic.velocity.y, kinematic.velocity.x);

    if (!texture.loadFromFile("boid-sm.png")) {
        sf::Image img({16u, 16u}, sf::Color::Transparent); // Fixed constructor: size as Vector2u, color Transparent
        for (unsigned y = 0; y < 16; ++y) {
            for (unsigned x = 0; x < 16; ++x) {
                img.setPixel({x, y}, sf::Color(180, 180, 180)); // Fixed: use Vector2u {x,y}
            }
        }
        (void)texture.loadFromImage(img); // Ignore nodiscard with (void)
    }
    sprite = std::make_unique<sf::Sprite>(texture);
    sprite->setOrigin({texture.getSize().x / 2.f, texture.getSize().y / 2.f}); // Fixed: use Vector2f {}
    sprite->setScale({1.5f, 1.5f});
    sprite->setColor(color);
}

void Character::setBehavior(SteeringBehavior *b) { currentBehavior = b; }
Kinematic &Character::getKinematic() { return kinematic; }
void Character::clearBreadcrumbs() { breadcrumbs.clear(); }
void Character::setMaxSpeed(float s) { maxSpeed = s; }
void Character::setPosition(sf::Vector2f p) { kinematic.position = p; sprite->setPosition(p); }

void Character::update(float dt, const Kinematic &target) {
    if (currentBehavior) {
        SteeringOutput s = currentBehavior->calculateSteering(kinematic, target);
        kinematic.position += kinematic.velocity * dt;
        kinematic.orientation += kinematic.rotation * dt;
        kinematic.velocity += s.linear * dt;
        kinematic.rotation += s.angular * dt;

        float speed = std::sqrt(kinematic.velocity.x*kinematic.velocity.x + kinematic.velocity.y*kinematic.velocity.y);
        if (speed > maxSpeed) kinematic.velocity = (kinematic.velocity / speed) * maxSpeed;

        if (std::abs(kinematic.rotation) > maxRotation)
            kinematic.rotation = (kinematic.rotation / std::abs(kinematic.rotation)) * maxRotation;
    } else {
        kinematic.position += kinematic.velocity * dt;
        kinematic.orientation += kinematic.rotation * dt;
    }

    kinematic.orientation = mapToRange(kinematic.orientation);
    breadcrumbs.update(kinematic.position);
    sprite->setPosition(kinematic.position);
    sprite->setRotation(sf::degrees(kinematic.orientation * 180.f / PI));
}

void Character::updateWithBoundaryHandling(float dt, const Kinematic &target) {
    (void)dt; // Suppress unused
    (void)target; // Suppress unused
    // Implement if needed from HW2; for now, placeholder or copy full if available
    // Assuming it's similar to update, with boundaries
    if (currentBehavior) {
        SteeringOutput s = currentBehavior->calculateSteering(kinematic, target);
        kinematic.position += kinematic.velocity * dt;
        kinematic.orientation += kinematic.rotation * dt;
        kinematic.velocity += s.linear * dt;
        kinematic.rotation += s.angular * dt;

        float speed = std::sqrt(kinematic.velocity.x*kinematic.velocity.x + kinematic.velocity.y*kinematic.velocity.y);
        if (speed > maxSpeed) kinematic.velocity = (kinematic.velocity / speed) * maxSpeed;

        if (std::abs(kinematic.rotation) > maxRotation)
            kinematic.rotation = (kinematic.rotation / std::abs(kinematic.rotation)) * maxRotation;
    } else {
        kinematic.position += kinematic.velocity * dt;
        kinematic.orientation += kinematic.rotation * dt;
    }

    const float margin = 4.f;
    if (kinematic.position.x < margin) { kinematic.position.x = margin; kinematic.velocity.x = std::abs(kinematic.velocity.x) * 0.8f; }
    if (kinematic.position.x > WINDOW_WIDTH - margin) { kinematic.position.x = WINDOW_WIDTH - margin; kinematic.velocity.x = -std::abs(kinematic.velocity.x) * 0.8f; }
    if (kinematic.position.y < margin) { kinematic.position.y = margin; kinematic.velocity.y = std::abs(kinematic.velocity.y) * 0.8f; }
    if (kinematic.position.y > WINDOW_HEIGHT - margin) { kinematic.position.y = WINDOW_HEIGHT - margin; kinematic.velocity.y = -std::abs(kinematic.velocity.y) * 0.8f; }

    kinematic.orientation = mapToRange(kinematic.orientation);
    breadcrumbs.update(kinematic.position);
    sprite->setPosition(kinematic.position);
    sprite->setRotation(sf::degrees(kinematic.orientation * 180.f / PI));
}

void Character::draw(sf::RenderWindow &win) {
    breadcrumbs.draw(win);
    win.draw(*sprite);
}

void Character::followPath(const std::vector<sf::Vector2f>& path, float dt) {
    if (path.empty()) return;
    static size_t idx = 0;
    Kinematic target;
    target.position = path[idx];
    update(dt, target);
    if (std::hypot(kinematic.position.x - path[idx].x, kinematic.position.y - path[idx].y) < 10.f) {
        idx++;
        if (idx >= path.size()) idx = 0;
    }
}