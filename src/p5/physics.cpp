#include "p5/physics.hpp"

namespace _462 {

Physics::Physics()
{
    reset();
}

Physics::~Physics()
{
    reset();
}

Derivative Physics::evaluate( SphereBody *sphere, real_t dt, const Derivative &d ) const
{
//    Vector3 x = sphere->position + (d.dx * dt);
//    Vector3 x = sphere->step_orientation( dt, 0.0f);
    Vector3 v = sphere->velocity + (d.dv * dt);

    Derivative output;
    output.dx = v;
    output.dv = sphere->force / sphere->mass;
    // output.dv = sphere->step_position(dt, 0.0f);

    return output;
}

void Physics::step( real_t dt )
{
    // TODO step the world forward by dt. Need to detect collisions, apply
    // forces, and integrate positions and orientations.
    //
    // Note: put RK4 here, not in any of the physics bodies
    //
    // Must use the functions that you implemented
    //
    // Note, when you change the position/orientation of a physics object,
    // change the position/orientation of the graphical object that represents
    // it

    std::vector< SphereBody* >::iterator it;
    std::vector< SphereBody* >::iterator it2;
    std::vector< PlaneBody* >::iterator pt;
    std::vector< TriangleBody* >::iterator tt;

    for (it = spheres.begin(); it != spheres.end(); ++it)
      {
        Derivative a,b,c,d;
        SphereBody *sb = *it;

      for (it2 = spheres.begin(); it2 != spheres.end(); ++it2)
        {
          if (*it != *it2 && collides(**it, **it2, collision_damping))
            {
              SphereBody *sb2 = *it2;

              Vector3 v1 = sb->velocity - sb2->velocity;
              Vector3 d = ((sb2->position - sb->position)
                  / distance(sb->position, sb2->position));
              Vector3 v22 = 2 * d
                * (sb->mass / (sb->mass + sb2->mass))
                * (dot(v1, d));
              Vector3 u2 = sb2->velocity + v22;
              Vector3 u1 =
                ((sb->mass * sb->velocity)
                + (sb2->mass * sb2->velocity)
                - (sb2->mass * u2))
                / sb->mass;

              sb->velocity = u1 - collision_damping * u1;
              sb2->velocity = u2 - collision_damping * u2;
            }
        }

    for (pt = planes.begin(); pt != planes.end(); ++pt)
      {
        if (collides(*sb, **pt, collision_damping))
          {
            Vector3 u = sb->velocity - 2 * dot(sb->velocity, (*pt)->normal) * (*pt)->normal;
            sb->velocity = u - collision_damping * u;
          }
      }

    for (tt = triangles.begin(); tt != triangles.end(); ++tt)
      {
        if (collides(*sb, **tt, collision_damping))
          {
            std::cout << "collision" << std::endl;
            Vector3 n =
              normalize(
                  cross(
                    (*tt)->vertices[0] - (*tt)->vertices[1],
                    (*tt)->vertices[1] - (*tt)->vertices[2]
                    )
                  );
            Vector3 u = sb->velocity - 2 * dot(sb->velocity, n) * n;
            sb->velocity = u - (collision_damping * u);
          }
      }

        sb->apply_force(gravity, Vector3::Zero());

        a = evaluate( sb, 0.0f, Derivative() );
        b = evaluate( sb, dt*0.5f, a );
        c = evaluate( sb, dt*0.5f, b );
        d = evaluate( sb, dt, c );

        Vector3 dxdt = ( a.dx + 2.0f*(b.dx + c.dx) + d.dx ) * (1.0f / 6.0f);

        Vector3 dvdt =
          ( a.dv + 2.0f*(b.dv + c.dv) + d.dv ) * (1.0f / 6.0f);

        sb->position += dxdt * dt;
        sb->sphere->position = sb->position;
        sb->velocity += dvdt * dt;
      }
}

void Physics::add_sphere( SphereBody* b )
{
    spheres.push_back( b );
}

size_t Physics::num_spheres() const
{
    return spheres.size();
}

void Physics::add_plane( PlaneBody* p )
{
    planes.push_back( p );
}

size_t Physics::num_planes() const
{
    return planes.size();
}

void Physics::add_triangle( TriangleBody* t )
{
    triangles.push_back( t );
}

size_t Physics::num_triangles() const
{
    return triangles.size();
}

void Physics::add_spring( Spring* s )
{
    springs.push_back( s );
}

size_t Physics::num_springs() const
{
    return springs.size();
}

void Physics::reset()
{
    for ( SphereList::iterator i = spheres.begin(); i != spheres.end(); i++ ) {
        delete *i;
    }
    for ( PlaneList::iterator i = planes.begin(); i != planes.end(); i++ ) {
        delete *i;
    }
    for ( TriangleList::iterator i = triangles.begin(); i != triangles.end(); i++ ) {
        delete *i;
    }
    for ( SpringList::iterator i = springs.begin(); i != springs.end(); i++ ) {
        delete *i;
    }

    spheres.clear();
    planes.clear();
    triangles.clear();
    springs.clear();

    gravity = Vector3::Zero();
    collision_damping = 0.0;
}

}
