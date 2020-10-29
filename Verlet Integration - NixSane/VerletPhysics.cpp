#include "VerletPhysics.h"

VerletPhysics::VerletPhysics(glm::vec2 position, int size)
{
	// Create the particles
	for (int i = 0; i < size; i++)
	{
		Point* P = new Point();
		particles.push_back(P);
		particles[i]->position = position + glm::vec2(2.0f * i, 0.0f);
		particles[i]->old_position = particles[i]->position;
		particles[i]->acceleration = glm::vec2(0);
		// Acceleration is default
	}

	// Create the Edges
	for (int iter = 0; iter < particles.size() - 1; iter++)
	{
		Edge* E = new Edge();
		edges.push_back(E);
	}

	// Fill the edges
	for (int j = 0; j < edges.size(); j++)
	{
		edges[j]->particle_1 = particles[j];
		edges[j]->particle_2 = particles[j + 1];
		edges[j]->rest_length = glm::length(edges[j]->particle_2->position
			- edges[j]->particle_1->position);
	}
}

VerletPhysics::VerletPhysics(glm::vec2 position, int row, int column, float spacing)
{
	// Make the rows
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < column; j++)
		{
			Point* P = new Point();
			P->position = position + glm::vec2(spacing * i, spacing * j);
			P->old_position = P->position;
			P->acceleration = glm::vec2(0);
			particles.push_back(P);
		}
	}

	// Main body
	main_body = particles[particles.size() * 0.5f];

	// Fill the horizontals
	for (int j = 0; j < row; j++)
	{
		for (int i = 0; i < column - 1; i++)
		{
			Edge* E = new Edge();

			E->particle_1 = particles[j * column + i];
			E->particle_2 = particles[j * column + i + 1];
			E->rest_length = length(E->particle_2->position
				- E->particle_1->position);

			edges.push_back(E);
		}
	}

	// Verticals 
	for (int i = 0; i < row - 1; ++i)
	{
		for (int j = 0; j < column; ++j)
		{
			Edge* E = new Edge();

			E->particle_1 = particles[i * column + j];
			E->particle_2 = particles[(i + 1) * column + j];
			E->rest_length = length(E->particle_2->position
				- E->particle_1->position);

			edges.push_back(E);
		}
	}
}

void VerletPhysics::setMainBody(int index)
{
	main_body = particles[index];
}

VerletPhysics::~VerletPhysics()
{
	for (Point* point : particles)
	{
		point = nullptr;
		delete point;
	}

	for (Edge* edge : edges)
	{
		edge = nullptr;
		delete edge;
	}
}

void VerletPhysics::verletUpdate(float m_time_step)
{
	for (int i = 0; i < particles.size(); i++)
	{
		glm::vec2& current_pos = particles[i]->position;
		glm::vec2& old_pos = particles[i]->old_position;
		glm::vec2& acc = particles[i]->acceleration;

		glm::vec2 temp = current_pos;
		current_pos += (current_pos - old_pos) + acc * (m_time_step * m_time_step);
		old_pos = temp;
	}
	constraits();
}

void VerletPhysics::constraits()
{
	for (int j = 0; j < edges.size(); j++)
	{
		Edge& edge = *edges[j];

		glm::vec2& x1 = edge.particle_1->position;
		glm::vec2& x2 = edge.particle_2->position;
		float inv_mass = 1.0f / edge.particle_1->mass;
		float inv_mass2 = 1.0f / edge.particle_2->mass;


		glm::vec2 vecBetweenPoints = x2 - x1;
		float vecFrom1to2 = glm::length(vecBetweenPoints);
		if (vecFrom1to2 > edges[j]->rest_length)
		{
			// Under tension, points should move towards each other.
			float error = vecFrom1to2 - edges[j]->rest_length;
			vecBetweenPoints = glm::normalize(vecBetweenPoints);

			//They should be moving towards each other
			x1 += (error * vecBetweenPoints) * 0.5f;
			x2 -= (error * vecBetweenPoints) * 0.5f;
		}
	}
}

void VerletPhysics::setAcceleration(int particle_index, float x_vel, float y_vel)
{
	particles[particle_index]->acceleration = glm::vec2(x_vel, y_vel);
}

void VerletPhysics::setAccelerationAll(float x_vel, float y_vel)
{
	for (int i = 0; i < particles.size(); i++)
	{
		particles[i]->acceleration = glm::vec2(x_vel, y_vel);
	}
}

void VerletPhysics::setPosition(glm::vec2 a_pos)
{
	particles[2]->old_position = particles[2]->position;
	particles[2]->position += a_pos;
}

glm::vec2 VerletPhysics::getPosition(int index)
{
	return particles[index]->position;
}

int VerletPhysics::getParticleCount()
{
	return particles.size();
}

int VerletPhysics::getEdgeCount()
{
	return edges.size();
}

Edge VerletPhysics::getEdges(int index)
{
	return *edges[index];
}