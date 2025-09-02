#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

// Forward declaration
void runPart2();

// --- Part 1 Logic (Unchanged) ---
void runPart1() {
    sf::RenderWindow window(sf::VideoMode({640, 480}), "Part 1: Simple Movement");
    window.setFramerateLimit(60);
    sf::Texture boidTexture;
    if (!boidTexture.loadFromFile("boid-ss.png")) {
        std::cerr << "Part 1 Error: Could not load boid-ss.png" << std::endl;
        return;
    }
    sf::Sprite boidSprite(boidTexture);
    boidSprite.setPosition({0.f, 0.f});
    float moveSpeed = 2.0f;
    while (window.isOpen()) {
        while (const auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
        }
        boidSprite.move({moveSpeed, 0.f});
        if (boidSprite.getPosition().x > window.getSize().x) {
            float spriteWidth = boidSprite.getGlobalBounds().size.x;
            boidSprite.setPosition({-spriteWidth, 0.f});
        }
        window.clear(sf::Color::White);
        window.draw(boidSprite);
        window.display();
    }
}

// --- Part 2 Logic ---

enum class Direction { RIGHT, DOWN, LEFT, UP };

struct Boid {
    sf::Sprite sprite;
    Direction dir;
    bool isTurning = false;
    int turnProgress = 0;
    float startRotation = 0.f;
    float targetRotation = 0.f;
    sf::Vector2f velocity = {0.f, 0.f};

    explicit Boid(const sf::Texture& texture) : sprite(texture), dir(Direction::RIGHT) {}
};

// --- FIXED: The synchronization logic is now more robust ---
void synchronizeBoids(std::vector<Boid>& boids, const sf::Vector2f& topRight, const sf::Vector2f& bottomRight, const sf::Vector2f& bottomLeft, const sf::Vector2f& topLeft) {
    if (boids.size() <= 1) return;

    const auto& leader = boids.front();

    // Correctly calculate the leader's remaining distance and speed magnitude
    float remainingDistLeader = 0;
    const auto& posLeader = leader.sprite.getPosition();
    switch (leader.dir) {
        case Direction::RIGHT: remainingDistLeader = topRight.x - posLeader.x; break;
        case Direction::DOWN:  remainingDistLeader = bottomRight.y - posLeader.y; break;
        case Direction::LEFT:  remainingDistLeader = posLeader.x - bottomLeft.x; break;
        case Direction::UP:    remainingDistLeader = posLeader.y - topLeft.y; break;
    }

    float speedMagLeader = std::sqrt(leader.velocity.x * leader.velocity.x + leader.velocity.y * leader.velocity.y);

    if (speedMagLeader <= 0) return; // Cannot sync if leader is stopped

    float timeToNextCorner = remainingDistLeader / speedMagLeader;

    for (size_t i = 1; i < boids.size(); ++i) {
        auto& follower = boids[i];
        if (follower.isTurning) continue;

        float remainingDistFollower = 0;
        const auto& posFollower = follower.sprite.getPosition();
        switch (follower.dir) {
            case Direction::RIGHT: remainingDistFollower = topRight.x - posFollower.x; break;
            case Direction::DOWN:  remainingDistFollower = bottomRight.y - posFollower.y; break;
            case Direction::LEFT:  remainingDistFollower = posFollower.x - bottomLeft.x; break;
            case Direction::UP:    remainingDistFollower = posFollower.y - topLeft.y; break;
        }
        
        float newSpeedMag = (timeToNextCorner > 0) ? (remainingDistFollower / timeToNextCorner) : 0;
        
        switch (follower.dir) {
            case Direction::RIGHT: follower.velocity = {newSpeedMag, 0.f}; break;
            case Direction::DOWN:  follower.velocity = {0.f, newSpeedMag}; break;
            case Direction::LEFT:  follower.velocity = {-newSpeedMag, 0.f}; break;
            case Direction::UP:    follower.velocity = {0.f, -newSpeedMag}; break;
        }
    }
}


void runPart2() {
    const unsigned int WIN_WIDTH = 640;
    const unsigned int WIN_HEIGHT = 480;
    const float PADDING = 50.0f;
    const int TURN_FRAMES = 30;

    sf::RenderWindow window(sf::VideoMode({WIN_WIDTH, WIN_HEIGHT}), "Part 2: Coordinated Movement");
    window.setFramerateLimit(60);

    sf::Texture boidTexture;
    if (!boidTexture.loadFromFile("boid-ss.png")) {
        std::cerr << "Part 2 Error: Could not load boid-ss.png" << std::endl;
        return;
    }
    boidTexture.setSmooth(true);

    const sf::Vector2f topLeft = {PADDING, PADDING};
    const sf::Vector2f topRight = {WIN_WIDTH - PADDING, PADDING};
    const sf::Vector2f bottomRight = {WIN_WIDTH - PADDING, WIN_HEIGHT - PADDING};
    const sf::Vector2f bottomLeft = {PADDING, WIN_HEIGHT - PADDING};

    std::vector<Boid> boids;
    const float baseSpeed = 2.0f;

    // --- FIXED: Use these variables to calculate speeds and remove warnings ---
    const float xDistance = topRight.x - topLeft.x;
    const float yDistance = bottomRight.y - topRight.y;
    const sf::Vector2f speedRight = {baseSpeed, 0.f};
    const sf::Vector2f speedDown = {0.f, baseSpeed * (yDistance / xDistance)};
    const sf::Vector2f speedLeft = {-baseSpeed, 0.f};
    const sf::Vector2f speedUp = {0.f, -baseSpeed * (yDistance / xDistance)};

    bool canAddBoids = true;

    auto addBoid = [&]() {
        Boid newBoid(boidTexture);
        newBoid.sprite.setOrigin(newBoid.sprite.getGlobalBounds().size / 2.f);
        newBoid.sprite.setPosition(topLeft);
        newBoid.velocity = speedRight;
        boids.push_back(newBoid);
    };

    addBoid();

    while (window.isOpen()) {
        while (const auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
        }

        bool leaderFinishedTurn = false;
        bool shouldAddBoid = false;

        for (auto& boid : boids) {
            if (boid.isTurning) {
                boid.turnProgress++;
                float t = static_cast<float>(boid.turnProgress) / TURN_FRAMES;
                float currentAngle = boid.startRotation + t * (boid.targetRotation - boid.startRotation);
                boid.sprite.setRotation(sf::degrees(currentAngle));

                if (boid.turnProgress >= TURN_FRAMES) {
                    boid.isTurning = false;
                    boid.turnProgress = 0;
                    boid.sprite.setRotation(sf::degrees(boid.targetRotation));
                    
                    switch(boid.dir) {
                        case Direction::DOWN: boid.velocity = speedDown; break;
                        case Direction::LEFT: boid.velocity = speedLeft; break;
                        case Direction::UP:   boid.velocity = speedUp; break;
                        case Direction::RIGHT:boid.velocity = speedRight; break;
                    }
                    
                    if (!boids.empty() && &boid == &boids.front()) {
                        leaderFinishedTurn = true;
                        if (canAddBoids) {
                            shouldAddBoid = true;
                        }
                    }
                }
            } else { 
                boid.sprite.move(boid.velocity);

                switch (boid.dir) {
                    case Direction::RIGHT:
                        if (boid.sprite.getPosition().x >= topRight.x) {
                            boid.sprite.setPosition(topRight); boid.dir = Direction::DOWN;
                            boid.isTurning = true; boid.velocity = {0,0};
                            boid.startRotation = 0.f; boid.targetRotation = 90.f;
                        }
                        break;
                    case Direction::DOWN:
                        if (boid.sprite.getPosition().y >= bottomRight.y) {
                            boid.sprite.setPosition(bottomRight); boid.dir = Direction::LEFT;
                            boid.isTurning = true; boid.velocity = {0,0};
                            boid.startRotation = 90.f; boid.targetRotation = 180.f;
                        }
                        break;
                    case Direction::LEFT:
                        if (boid.sprite.getPosition().x <= bottomLeft.x) {
                            boid.sprite.setPosition(bottomLeft); boid.dir = Direction::UP;
                            boid.isTurning = true; boid.velocity = {0,0};
                            boid.startRotation = 180.f; boid.targetRotation = 270.f;
                        }
                        break;
                    case Direction::UP:
                        if (boid.sprite.getPosition().y <= topLeft.y) {
                            boid.sprite.setPosition({-1000.f, -1000.f});
                        }
                        break;
                }
            }
        }
        
        if (leaderFinishedTurn) {
            synchronizeBoids(boids, topRight, bottomRight, bottomLeft, topLeft);
        }

        if (shouldAddBoid) {
            addBoid();
            if (boids.size() == 4) {
                canAddBoids = false;
            }
        }
        
        bool leaderWasRemoved = false;
        boids.erase(std::remove_if(boids.begin(), boids.end(),
            [&](const Boid& b) {
                if (b.sprite.getPosition().x < 0) {
                    if (!boids.empty() && &b == &boids.front()) {
                        leaderWasRemoved = true;
                    }
                    return true;
                }
                return false;
            }), boids.end());
        
        if (leaderWasRemoved) {
            synchronizeBoids(boids, topRight, bottomRight, bottomLeft, topLeft);
        }

        if (boids.empty()) {
            canAddBoids = true;
            addBoid();
        }

        window.clear(sf::Color::White);
        for (const auto& boid : boids) window.draw(boid.sprite);
        window.display();
    }
}

// --- Main Function (Unchanged) ---
int main() {
    std::cout << "Starting Part 1... Close the window to continue to Part 2." << std::endl;
    runPart1();
    std::cout << "Part 1 finished. Starting Part 2..." << std::endl;
    runPart2();
    std::cout << "Part 2 finished. Exiting program." << std::endl;
    return 0;
}