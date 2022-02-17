#pragma once

// c for components
namespace c
{
	// The transform component
	struct Transform
	{
		Transform() = default;

		sf::Vector2f position = sf::Vector2f(0.f, 0.f);
		sf::Vector2f velocity = sf::Vector2f(0.f, 0.f);
		float radius = 0.f;
	};

	struct RenderData
	{
		RenderData() = default;
	};
}

// s for systems
namespace s
{

};

// eps for extra parameter systems
namespace eps
{
	static void BoidsAlgorithm(ECS& ecs, float DeltaTime, sf::RenderWindow* window)
	{
		// A boid's velocity will be taken as their forward vector

		// Boid const variables
		const float maxSpeed = 200.f;
		const float minSpeed = 100.f;
		const float seperationDist = 20.f;
		const float seperationDistSquared = seperationDist * seperationDist;
		const float seperationVelocity = 20.f;
		const float alingmentDist = 100.f;
		const float alignmentDistSquared = alingmentDist * alingmentDist;
		const float alignmentFactor = 0.05f;
		const float coherenceFactor = 0.04f;
		const float barrierAvoidanceRange = 20.f;
		const float barrierAvoidanceVel = 20.f;

		// Get relevent entities - visible to Lambdas
		auto entitiesWithComponents = ecs.getEntitiesWithComponents<c::Transform>();

		//// Helper lambdas
		//auto getAngle = [&](const sf::Vector2f& a, const sf::Vector2f& b) -> float
		//{
		//	const float dot = a.x * b.x + a.y * b.y;
		//	const float denominator = sqrtf(a.x * a.x + a.y * a.y) * sqrtf(b.x * b.x + b.y * b.y);
		//	return acos(dot / denominator);
		//};
		//auto rotateVector = [&](sf::Vector2f vector, const float angle)
		//{
		//	auto og = vector;
		//	vector.x = cosf(angle) * og.x - sinf(angle) * og.y;
		//	vector.y = sinf(angle) * og.x + cosf(angle) * og.y;
		//};

		// Applies the velocity to the position
		auto updatePositions = [&]()
		{
			for (auto& entityID : *entitiesWithComponents)
			{
				// Get component
				auto& transform = *ecs.getEntitysComponent<c::Transform>(entityID);

				// Process this component
				transform.position += transform.velocity * DeltaTime;
			}
		};

		// Runs the main boid algorithm
		auto calculateBoid = [&](EntityID entityID)
		{
			// Get this boid's transform
			auto thisTransform = ecs.getEntitysComponent<c::Transform>(entityID);

			// Loop through all entities to get close entities - the largest distance away needed
			std::vector<c::Transform*> closeEntities;
			for (auto& otherID : *entitiesWithComponents)
			{
				// If not this
				if (otherID == entityID)
					continue;

				// Get other transform
				auto otherTransform = ecs.getEntitysComponent<c::Transform>(otherID);

				// Test distance 
				const float distanceSquared = powf((thisTransform->position.x - otherTransform->position.x), 2.f) + powf((thisTransform->position.y - otherTransform->position.y), 2.f);
				if (distanceSquared <= alignmentDistSquared)
				{
					// Add to array
					closeEntities.push_back(otherTransform);
				}
			}

			// If there are no close boids, return
			if (closeEntities.size() == 0)
				return;

			// Coherence - Boids will steer closer towards other boids
			auto coherence = [&]() -> sf::Vector2f
			{
				sf::Vector2f averagePosition = sf::Vector2f(0.f, 0.f);
				for (auto transform : closeEntities)
				{
					averagePosition += transform->position;
				}
				averagePosition /= (float)closeEntities.size();

				return (averagePosition - thisTransform->position) * coherenceFactor;
			};

			// Seperation - Boids will steer away from other boids if they get too close
			auto seperation = [&]() -> sf::Vector2f
			{
				auto seperate = [&](const float pos, const float otherPos, const float percent, float& vel)
				{
					const auto direction = pos < otherPos ? 1.f : -1.f;

					vel -= seperationVelocity * percent * direction;
				};

				sf::Vector2f output = sf::Vector2f(0.f, 0.f);

				// Loop through the close entities to find the boids that are within seperation distance
				for (auto* otherTransform : closeEntities)
				{
					// Test distance 
					const float distanceSquared = powf((thisTransform->position.x - otherTransform->position.x), 2.f) + powf((thisTransform->position.y - otherTransform->position.y), 2.f);
					if (distanceSquared <= seperationDistSquared)
					{
						// Within distance to move away

						const auto percent = 1.f - (distanceSquared / seperationDistSquared);
						seperate(thisTransform->position.x, otherTransform->position.x, percent, output.x);
						seperate(thisTransform->position.y, otherTransform->position.y, percent, output.y);

						//output -= (otherTransform->position - thisTransform->position);
					}
				}

				return output;
			};

			// Alignment - Boids modify their velocity magnitude and direction to match that of adjacent boids
			auto alignment = [&]() -> sf::Vector2f
			{
				// Get average velocity and steer this velocity towards that. 

				// Average velocity
				sf::Vector2f averageVelocity = sf::Vector2f(0.f, 0.f);
				for (auto transform : closeEntities)
					averageVelocity += transform->velocity;
				averageVelocity /= (float)closeEntities.size();

				return (averageVelocity - thisTransform->velocity) * alignmentFactor;
			};

			// Barrier - Avoidance logic but with the edges of the window
			auto barrier = [&]() -> sf::Vector2f
			{
				sf::Vector2f output = sf::Vector2f(0.f, 0.f);

				auto testBorder = [&](float pos, float& vel, const float edge)
				{
					// Test left (in x's case)
					if (pos <= barrierAvoidanceRange)
						vel = barrierAvoidanceVel - pos;
					// Test right (in x's case)
					else if (pos >= edge - barrierAvoidanceRange)
						vel = -(barrierAvoidanceVel - (edge - pos));
				};

				testBorder(thisTransform->position.x, output.x, window->getSize().x); // Test x
				testBorder(thisTransform->position.y, output.y, window->getSize().y); // Test y

				return (output);
			};

			auto coher_vel = coherence();
			auto sep_vel = seperation();
			auto align_vel = alignment();
			auto barrier_vel = barrier();

			// Add velocities
			thisTransform->velocity = thisTransform->velocity + align_vel + sep_vel + coher_vel + barrier_vel;

			// Clamp speeds
			const auto speed = sqrtf((thisTransform->velocity.x * thisTransform->velocity.x) + (thisTransform->velocity.y * thisTransform->velocity.y));
			if (speed > maxSpeed)
				thisTransform->velocity *= maxSpeed / speed;
			else if (speed < minSpeed)
				thisTransform->velocity *= minSpeed / speed;
		};

		// Loop through entities
		for (auto& entityID : *entitiesWithComponents)
		{
			calculateBoid(entityID);
		}

		// Finally, update positions
		updatePositions();
	}

	static void renderBoid(ECS& ecs, float DeltaTime, sf::RenderWindow* window, sf::CircleShape& circle)
	{
		auto entitiesWithComponents = ecs.getEntitiesWithComponents<c::RenderData>();

		for (auto& entityID : *entitiesWithComponents)
		{
			// Get components
			auto* transform = ecs.getEntitysComponent<c::Transform>(entityID);
			auto* renderData = ecs.getEntitysComponent<c::RenderData>(entityID);

			// Process information
			circle.setPosition(transform->position);
			circle.setRadius(transform->radius);

			window->draw(circle);
		}
	}
};
