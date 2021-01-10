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
            masses[i]->mass = node_mass;
            masses[i]->position = start + (float)i / num_nodes * (end - start);
            masses[i]->start_position = masses[i]->position;
            masses[i]->last_position = masses[i]->position;
        }

        springs.resize(num_nodes - 1);
        for (size_t i = 0; i < num_nodes - 1; ++i)
        {
            springs[i]->k = k;
            springs[i]->m1 = masses[i];
            springs[i]->m2 = masses[i + 1];
            springs[i]->rest_length = (springs[i]->m1->position - springs[i]->m2->position).norm();
        }

        for (auto &i : pinned_nodes)
        {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        float k = springs[0]->k;
        for (auto &s : springs)
        {
            auto dis = (s->m2->position - s->m1->position).norm();
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            s->m1->forces = s->k * (s->m1->position - s->m2->position) / dis * (dis - s->rest_length);
            s->m2->forces = -s->m1->forces;
        }

        //对每段绳子只更新一段的质点的速度
        for (auto &s : springs)
        {
            auto m = s->m1;
            Vector2D f;
            if (!m->pinned)
            {
                //计算internal dumping
                Vector2D dir = (s->m1->position - s->m2->position) / (s->m1->position - s->m2->position).norm();
                f = -k * dir  * (s->m1->velocity - s->m2->velocity)* dir;

                //施加重力和摩擦内力
                m->forces += gravity;
                m->forces -= f;

                //更新位置
                m->velocity += m->forces / m->mass * delta_t;
                m->position += m->velocity * delta_t;
                m->forces = Vector2D(0, 0);
            }
            m = s->m2;
            if (!m->pinned)
            {
                m->forces += f;
            }
        }
        // for (auto &m : masses)
        // {
        //     if (!m->pinned)
        //     {
        //         // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
        //         m->forces += gravity;
        //         m->velocity += m->forces / m->mass * delta_t;
        //         m->position += m->velocity * delta_t;

        //         // TODO (Part 2): Add global damping
        //         //m->forces+=-k*
        //     }

        //     // Reset all forces on each mass
        //     m->forces = Vector2D(0, 0);
        // }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass

                // TODO (Part 4): Add global Verlet damping
            }
        }
    }
} // namespace CGL
