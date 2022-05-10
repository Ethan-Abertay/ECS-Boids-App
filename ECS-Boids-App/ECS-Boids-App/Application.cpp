#include "Application.h"
#include "ComponentsAndSystems.h"

Application::Application()
{
	// Create window
	window = new sf::RenderWindow(sf::VideoMode(2560, 1440), "Collision App", sf::Style::Fullscreen);

	// srand
	//srand(time(0));
	srand(0);

	// Get Font and text
	bool result = arialFont.loadFromFile("arial.ttf");
	fps_text.setFont(arialFont);
	fps_text.setCharacterSize(24); // in pixels, not points!
	fps_text.setFillColor(sf::Color::Red);
	fps_text.setStyle(sf::Text::Bold);

	// Initialise ECS
	ecs = new ECS();

	// Initialise components 
	ecs->initComponents<c::Transform, c::RenderData>();

	// Rand range
	auto randRange = [](float min, float max) -> float
	{
		auto normalizedFloat = (float)(rand()) / (float)(RAND_MAX);	// Between 0.f and 1.f
		return min + ((max - min) * normalizedFloat);
	};

	const float width = window->getSize().x;
	const float height = window->getSize().y;
	const float maxVel = 300.f;
	const float maxAcc = 25.f;
	const float minSize = 5.f;
	const float maxSize = 10.f;

	// Iniitalise entities
	for (int i = 0; i < 1000; ++i)
	{
		// Create entity - assign comps
		auto id = ecs->init_CreateEntity<c::Transform, c::RenderData>();

		// Randomise transform
		auto* transform = ecs->getEntitysComponent<c::Transform>(id);
		transform->position = sf::Vector2f(randRange(0, width), randRange(0, height));
		transform->velocity = sf::Vector2f(randRange(-maxVel, maxVel), randRange(-maxVel, maxVel));
		//transform->acceleration = sf::Vector2f(randRange(-maxAcc, maxAcc), randRange(-maxAcc, maxAcc));
		transform->radius = 5.f;
		//transform->size = sf::Vector2f(randRange(minSize, maxSize), randRange(minSize, maxSize));
	}

#if IMPL == 3

	ecs->performFullRefactor();

#endif

	// Create rectangle asset
	circle.setOutlineColor(sf::Color::Red);
	circle.setFillColor(sf::Color::White);
	circle.setOutlineThickness(0);
}

Application::~Application()
{
	if (window)
		delete window;
	window = 0;
}

void Application::run()
{
	// Length of time to run test in seconds
	float totalTime = 0.f;

	while (window->isOpen())
	{
		// Record start time
		auto start = Clock::now();

		// Run Game Loop
		gameLoop();

		// Record end time
		auto end = Clock::now();

		// Calculate Delta Time - Number of microseconds converted to float then divided by 1,000 twice to get delta time in seconds
		DeltaTime = (float)(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()) / 1000.f / 1000.f;

		// Calculate fps
		FPS = 1.f / DeltaTime;

		// Add to total time
		totalTime += DeltaTime;

		// Add delta time to record
		deltaTimes.push_back(std::pair(DeltaTime, totalTime));

		// Test if done (15 seconds)
		if (totalTime >= 15.f)
		{
			// Record output
			std::ofstream file;
			file.open("Output.csv");
			for (auto& f : deltaTimes)
				file << f.first << ',' << f.second << std::endl;
			file.close();

			// Close window (hence application)
			window->close();
		}

		//std::cout << "DeltaTime " << DeltaTime << " FPS " << FPS << std::endl;
	}
}

void Application::gameLoop()
{
	updateInputs();
	update();
	render();
}

void Application::updateInputs()
{
	while (window->pollEvent(e))
	{
		if (e.type == sf::Event::Closed)
			window->close();
	}
}

void Application::update()
{
	// Handle standard systems
	//ecs->processSystems<s::>(DeltaTime);

	// Handle extra systems
	eps::BoidsAlgorithm(*ecs, DeltaTime, window);
}

void Application::render()
{
	window->clear();

	eps::renderBoid(*ecs, DeltaTime, window, circle);

	std::string s = "FPS " + std::to_string((int)FPS);
	fps_text.setString(s);
	window->draw(fps_text);

	window->display();
}
