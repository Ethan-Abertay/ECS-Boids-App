#pragma once

// c for components
namespace c
{
	// The transform component
	struct Transform
	{
		Transform() = default;

		sf::Vector2f position = sf::Vector2f(0.f, 0.f);
		sf::Vector2f size = sf::Vector2f(80.f, 80.f);
		sf::Vector2f velocity = sf::Vector2f(0.f, 0.f);
		sf::Vector2f acceleration = sf::Vector2f(0.f, 0.f);
	};

	struct RenderData
	{
		RenderData() = default;

	};
}

// s for systems
namespace s
{
	struct Translation
	{
		static void process(ECS& ecs, float DeltaTime)
		{
			// Get relevent entities
			auto entitiesWithComponents = ecs.getEntitiesWithComponents<c::Transform>();

			// Loop through entities
			for (auto& entityID : *entitiesWithComponents)
			{
				// Get this entity's components
				auto* transform = ecs.getEntitysComponent<c::Transform>(entityID);

				// Process this component
				transform->velocity += transform->acceleration * DeltaTime;
				transform->position += transform->velocity * DeltaTime;
				//cout << position->position.x << endl;

				//cout << "Vel " << transform->velocity.x << endl;
			}
		}
	};

	struct EntityCollision
	{
		static void postCollision(float& vel1, float& acc1, float& vel2, float& acc2)
		{
			// Handle vel and acc
			if (vel1 * vel2 < 0)	// If travelling in opposite directions
			{
				vel1 *= -1.f;
				acc1 *= -1.f;
				vel2 *= -1.f;
				acc2 *= -1.f;
			}
			else	// Travelling in the same direction
			{
				// Invert the greater velocity
				if (abs(vel1) > abs(vel2))
				{
					vel1 *= -1.f;
					acc1 *= -1.f;
				}
				else
				{
					vel2 *= -1.f;
					acc2 *= -1.f;
				}
			}
		}

		static void handleCollision(c::Transform& transform1, c::Transform& transform2)
		{
			auto left1 = transform1.position.x;
			auto right1 = left1 + transform1.size.x;
			auto top1 = transform1.position.y;
			auto bottom1 = top1 + transform1.size.y;
			auto left2 = transform2.position.x;
			auto right2 = left2 + transform2.size.x;
			auto top2 = transform2.position.y;
			auto bottom2 = top2 + transform2.size.y;

			bool overlapX = (right1 > left2 && right1 < right2) || (left1 > left2 && left1 < right2) || (right2 > left1 && right2 < right1) || (left2 > left1 && left2 < right1);
			bool overlapY = (bottom1 > top2 && bottom1 < bottom2) || (top1 > top2 && top1 < bottom2) || (bottom2 > top1 && bottom2 < bottom1) || (top2 > top1 && top2 < bottom1);

			if (overlapX &&		// Collision X
				overlapY)		// Collision Y
			{
				// These entities are colliding

				// Displace 
				auto offsetRight = abs(right1 - left2);
				auto offsetLeft = abs(left1 - right2);
				auto offsetTop = abs(top1 - bottom2);
				auto offsetBottom = abs(bottom1 - top2);

				// Determine which direction to bounce
				bool collideRight = offsetRight < offsetLeft&& offsetRight < offsetTop&& offsetRight < offsetBottom;
				bool collideLeft = offsetLeft < offsetRight&& offsetLeft < offsetTop&& offsetLeft < offsetBottom;
				bool collideTop = offsetTop < offsetRight&& offsetTop < offsetLeft&& offsetTop < offsetBottom;
				bool collideBottom = offsetBottom < offsetRight&& offsetBottom < offsetLeft&& offsetBottom < offsetTop;

				//cout << "right " << collideRight << " left " << collideLeft << " top " << collideTop << " bottom " << collideBottom << endl;

				if (collideRight)
				{
					// Move
					transform1.position.x -= offsetRight / 2.f;
					transform2.position.x += offsetRight / 2.f;

					postCollision(transform1.velocity.x, transform1.acceleration.x, transform2.velocity.x, transform2.acceleration.x);
				}
				else if (collideLeft)
				{
					// Move
					transform1.position.x -= offsetLeft / 2.f;
					transform2.position.x += offsetLeft / 2.f;

					postCollision(transform1.velocity.x, transform1.acceleration.x, transform2.velocity.x, transform2.acceleration.x);
				}
				else if (collideTop)
				{
					// Move
					transform1.position.y -= offsetTop / 2.f;
					transform2.position.y += offsetTop / 2.f;

					postCollision(transform1.velocity.y, transform1.acceleration.y, transform2.velocity.y, transform2.acceleration.y);
				}
				else if (collideBottom)
				{
					// Move
					transform1.position.y -= offsetBottom / 2.f;
					transform2.position.y += offsetBottom / 2.f;

					postCollision(transform1.velocity.y, transform1.acceleration.y, transform2.velocity.y, transform2.acceleration.y);
				}
			}
		}

		static void process(ECS& ecs, float DeltaTime)
		{
			//return;

			// Get relevent entities
			auto entitiesWithComponents = ecs.getEntitiesWithComponents<c::Transform>();

			// Loop through entities
			for (int i = 0; i < entitiesWithComponents->size(); ++i)
			{
				for (int j = i + 1; j < entitiesWithComponents->size(); ++j)
				{
					auto* transform1 = ecs.getEntitysComponent<c::Transform>(entitiesWithComponents->at(i));
					auto* transform2 = ecs.getEntitysComponent<c::Transform>(entitiesWithComponents->at(j));

					handleCollision(*transform1, *transform2);
				}
			}
		}
	};
};

// es for extra parameter systems
namespace eps
{
	static void checkBoundaryCollision(ECS& ecs, sf::RenderWindow* window)
	{
		// Lambda to handle boundaries
		auto process = [](float* pos, float* vel, float* acc, const float width, const float size)
		{
			if (*pos < 0)
			{
				*pos = 0;
				*vel *= -1.f;
				*acc *= -1.f;
			}
			else if (*pos + width >= size)
			{
				*pos = size - width;
				*vel *= -1.f;
				*acc *= -1.f;
			}
		};

		// Get relevent entities
		auto entitiesWithComponents = ecs.getEntitiesWithComponents<c::Transform, c::RenderData>();

		// Loop through entities
		for (auto& entityID : *entitiesWithComponents)
		{
			// Get components
			auto* transform = ecs.getEntitysComponent<c::Transform>(entityID);

			// Get variables
			auto* position = &transform->position;
			auto* size = &transform->size;

			// Process variables
			process(&position->x, &transform->velocity.x, &transform->acceleration.x, size->x, window->getSize().x);
			process(&position->y, &transform->velocity.y, &transform->acceleration.y, size->y, window->getSize().y);
		}
	}

	static void renderRectangle(ECS& ecs, float DeltaTime, sf::RenderWindow* window, sf::RectangleShape& rectangle)
	{
		auto entitiesWithComponents = ecs.getEntitiesWithComponents<c::RenderData>();

		for (auto& entityID : *entitiesWithComponents)
		{
			// Get components
			auto* transform = ecs.getEntitysComponent<c::Transform>(entityID);
			auto* renderData = ecs.getEntitysComponent<c::RenderData>(entityID);

			// Process information
			rectangle.setPosition(transform->position);
			rectangle.setSize(transform->size);

			window->draw(rectangle);
		}
	}
};
