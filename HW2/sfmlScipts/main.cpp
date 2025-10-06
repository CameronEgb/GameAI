// #pragma once

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

/// <summary> 
/// Moves a sprite across the screen from left to right. Once the sprite goes off the right-side border, 
/// resets back to the start point. 
/// </summary>
void runPart2() {
    // Create a window with VGA resolution (640x480)
    sf::RenderWindow window(sf::VideoMode({640, 480}), "Homework 01 Part 2: Left-to-Right movement");
    window.setFramerateLimit(100);                              // Setting 100 FPS

    // Load the image file into a texture.
    sf::Texture texture;
    if (!texture.loadFromFile("boid-sm.png")) {
        std::cerr << "Error: Could not load boid-sm.png!" << std::endl;
        return;
    }

    // Create a sprite using the texture (vertical 10 pixel offset to make it fully visisble)
    sf::Sprite sprite(texture);
    sprite.setPosition({0.f, 10.f}); 

    // Run the main loop
    while (window.isOpen()) {
        while (auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        // Constantly move the sprite towards righthand-side by 2 pixel. Resets if out-of-screen 
        sprite.move({2.0f, 0.0f}); 
        if (sprite.getPosition().x > 640) { 
            sprite.setPosition({0.f, 10.f}); 
        }

        // Render the sprite on a white color background so it will be visible
        window.clear(sf::Color::White); 
        window.draw(sprite);
        window.display();
    }
}

/// <summary>
/// Moves sprites along the edges of the screen in a synchronous manner. Only four sprites max at any given time. 
/// Once a sprite reaches the starting point, it dissapears. Once all four reach the starting point, the movement 
/// restarts. 
/// </summary>
void runPart3() {
    // Create a window with VGA resolution (640x480)
    sf::RenderWindow window(sf::VideoMode({640, 480}), "Homework 01 Part 3: Complex movement");
    window.setFramerateLimit(100);                              // Setting 100 FPS

    // Load the image file into a texture.
    sf::Texture texture;
    if (!texture.loadFromFile("boid-sm.png")) {
        std::cerr << "Error: Could not load boid-sm.png!" << std::endl;
        return;
    }

    // Constants for distances. Horizontal distances are larger than vertical because of uneven window
    // size. The 20 pixel deduction is to account for the offsets left for visibility.
    const float horizontalDistance = 640 - 20;                
    const float verticalDistance = 480 - 20;                  
    const float baseSpeed = 2.0f;                             

    // Calculate synchronized speeds since vertical and horizontal speeds need to be different
    const float horizontalSpeed = baseSpeed;   
    const float verticalSpeed = baseSpeed * (verticalDistance / horizontalDistance); 

    // Structure to hold the sprites and variables related to them.
    struct SpriteData {
        sf::Sprite sprite;
        sf::Vector2f velocity;                                // Current speed of the sprite
        int rotationState;                                    // 0: Right, 1: Down, 2: Left, 3: Up
        bool active;                                          // Whether the sprite is still active
    };

    std::vector<SpriteData> sprites;

    // Add the first sprite into the list. First one starts at top-left corner.
    sprites.push_back({ sf::Sprite(texture), sf::Vector2f(horizontalSpeed, 0.0f), 0, true });
    sprites[0].sprite.setPosition({10.f, 10.f}); 

    // Run the main loop
    while (window.isOpen()) {
        while (auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        // Temporary list to store the dynamically created new sprites 
        std::vector<SpriteData> newSprites;

        // Move the sprites if they are in the 'active' state.
        for (auto& data : sprites) {
            if (!data.active)
                continue;

            data.sprite.move(data.velocity); 

            // Handles the bahaviour when a sprite reaches top-right corner
            if (data.rotationState == 0 && data.sprite.getPosition().x > horizontalDistance) {                
                data.sprite.setRotation(sf::degrees(90));
                data.velocity = sf::Vector2f(0.0f, verticalSpeed); 
                data.rotationState++;
                if (sprites.size() < 4) {
                    newSprites.push_back({ sf::Sprite(texture), sf::Vector2f(horizontalSpeed, 0.0f), 0, true });
                    newSprites.back().sprite.setPosition({10.f, 10.f});
                }
            }
            // Handles the bahaviour when a sprite reaches bottom-right corner
            else if (data.rotationState == 1 && data.sprite.getPosition().y > verticalDistance) {                
                data.sprite.setRotation(sf::degrees(180));
                data.velocity = sf::Vector2f(-horizontalSpeed, 0.0f); 
                data.rotationState++;
            }
            // Handles the bahaviour when a sprite reaches bottom-left corner
            else if (data.rotationState == 2 && data.sprite.getPosition().x < 10) {                
                data.sprite.setRotation(sf::degrees(270));
                data.velocity = sf::Vector2f(0.0f, -verticalSpeed); 
                data.rotationState++;
            }
            // Handles the bahaviour when a sprite reaches top-left corner
            else if (data.rotationState == 3 && data.sprite.getPosition().y < 10) {                
                data.sprite.setRotation(sf::degrees(0));
                data.velocity = sf::Vector2f(horizontalSpeed, 0.0f); 
                data.rotationState = 0;
                
                data.active = false;                          // Makes the sprite inactive       
            }
        }

        // Add any newly created sprites into the main list
        sprites.insert(sprites.end(), newSprites.begin(), newSprites.end());

        // Restart if all sprites are inactive (meaning all four reached the starting point)
        if (std::all_of(sprites.begin(), sprites.end(), [](const SpriteData& data) { return !data.active; })) {
            sprites.clear(); 
            sprites.push_back({ sf::Sprite(texture), sf::Vector2f(horizontalSpeed, 0.0f), 0, true });
            sprites[0].sprite.setPosition({10.f, 10.f}); 
        }

        // Only render the 'active' sprites on a white color background.
        window.clear(sf::Color::White); 
        for (const auto& data : sprites) {
            if (data.active)
                window.draw(data.sprite);
        }
        window.display();
    }
}


/// <summary>
/// Main function to run the code. Takes in an commandline argument to decide which behaviour to execute.
/// </summary>
int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <part2|part3>" << std::endl;
        return 1;
    }

    std::string option = argv[1];

    if (option == "part2") {
        runPart2();
    }
    else if (option == "part3") {
        runPart3();
    }
    else {
        std::cerr << "Invalid commandline argument. Use 'part2' or 'part3'. E.g., ./main part2" << std::endl;
        return 1;
    }

    return 0;
}
