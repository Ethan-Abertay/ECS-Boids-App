#pragma once

#include <iostream>
#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "SFML/Graphics.hpp"
#include "../../../ECS-Library/ECS/ECS/ECS.h"

typedef std::chrono::high_resolution_clock Clock;

class Application
{
public:
	Application();
	~Application();
	void run();

private:
	// Functions
	void gameLoop();
	void updateInputs();
	void update();
	void render();

	// Classes
	sf::RenderWindow* window = 0;
	sf::Event e;
	ECS* ecs = 0;
	sf::RectangleShape rectangle;
	sf::Text fps_text;
	sf::Font arialFont;

	// Variables
	float DeltaTime = 0.f;
	float FPS = 0.f;
};
