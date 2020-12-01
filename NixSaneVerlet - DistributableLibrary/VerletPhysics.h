#include "glm/glm.hpp"
#include <vector>

// Particle
struct Point
{
	glm::vec2 position = glm::vec2(-5.0f, 0.0f);
	glm::vec2 old_position = position;
	glm::vec2 acceleration = glm::vec2(0.0f, 0.0f);
	float mass = 1.0f;
};

// Draw a line from two points or Particles
struct Edge
{
	Point* particle_1;
	Point* particle_2;

	float rest_length = 2.0f; // When first made
};

#pragma once
class VerletPhysics
{
public:
	// Make Chain
	VerletPhysics(glm::vec2 position, int size);

	// Make a grid
	VerletPhysics(glm::vec2 position, int row, int column, float spacing);

	~VerletPhysics();

	// Set a particle as main body
	void setMainBody(int index);

	// Handles particle movement
	void verletUpdate(float m_time_step);

	// Keeps the points together
	void constraits();

	// Set acceleration for one particle
	void setAcceleration(int particle_index, float x_velocity, float y_velocity);

	void setAccelerationAll(float x_velocity, float y_velocity);

	// Set a position to move it to
	void setPosition(glm::vec2 a_position);

	Edge getEdges(int index);

	// Get particle position of specified particle
	glm::vec2 getPosition(int index);

	// Get size of the chain
	int getParticleCount();

	// Get number of edges
	int getEdgeCount();



private:
	std::vector<Point*> particles;
	size_t particle_size = 5;

	std::vector<Edge*> edges;
	size_t edge_size = 0;

	Point* main_body;
};

