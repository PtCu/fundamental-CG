#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL
{

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

        masses.resize(num_nodes);
        for (size_t i = 0; i < num_nodes; ++i)
        {
            masses[i] = new Mass(start + (float)i / num_nodes * (end - start), node_mass, false);
            masses[i]->start_position = masses[i]->position;
            masses[i]->last_position = masses[i]->position;
        }

        springs.resize(num_nodes - 1);
        for (size_t i = 0; i < num_nodes - 1; ++i)
        {
            springs[i] = new Spring(masses[i], masses[i + 1], k);
            springs[i]->rest_length = (springs[i]->m1->position - springs[i]->m2->position).norm();
        }

        for (auto &i : pinned_nodes)
        {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        
        float k = 0.01;
        for (auto &s : springs)
        {
            auto dis = (s->m2->position - s->m1->position).norm();
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            s->m1->forces = s->k * (s->m1->position - s->m2->position) / dis * (dis - s->rest_length);
            s->m2->forces = -s->m1->forces;

            //计算internal dumping
            Vector2D dir = (s->m1->position - s->m2->position) / dis;
            Vector2D f = -k * dir * (s->m1->velocity - s->m2->velocity) * dir;

            //施加摩擦内力
            s->m1->forces -= f;
           // std::cout<<"1"<<endl;
            s->m2->forces += f;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity;
                m->velocity += m->forces / m->mass * delta_t;
                m->position += m->velocity * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            auto dis = (s->m2->position - s->m1->position).norm();
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            s->m1->forces = s->k * (s->m1->position - s->m2->position) / dis * (dis - s->rest_length);
            s->m2->forces = -s->m1->forces;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                m->forces += gravity;
                Vector2D a = m->forces / m->mass;
                // TODO (Part 3.1): Set the new position of the rope mass
                Vector2D temp_position = m->position;
                float damp_factor = 0.00005;
                // TODO (Part 4): Add global Verlet damping
                m->position = m->position + (1 - damp_factor) * (m->position - m->last_position) + a * delta_t * delta_t;
                m->last_position = temp_position;
            }
        }
    }
} // namespace CGL
