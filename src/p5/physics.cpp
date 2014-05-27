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
    std::cout << "ACC: " << output.dv << std::endl;

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

    for (it = spheres.begin(); it != spheres.end(); ++it)
      {
        Derivative a,b,c,d;
        SphereBody *sb = *it;

        sb->apply_force(gravity, Vector3::Zero());

        a = evaluate( sb, 0.0f, Derivative() );
        b = evaluate( sb, dt*0.5f, a );
        c = evaluate( sb, dt*0.5f, b );
        d = evaluate( sb, dt, c );

        Vector3 dxdt = ( a.dx + 2.0f*(b.dx + c.dx) + d.dx ) * (1.0f / 6.0f);

        Vector3 dvdt =
          ( a.dv + 2.0f*(b.dv + c.dv) + d.dv ) * (1.0f / 6.0f);

        std::cout << "----------------------------" << std::endl;
        std::cout << "Position: " << sb->position << ", ";
        sb->position += dxdt * dt;
        sb->sphere->position = sb->position;
        std::cout << sb->position << std::endl << "Velocity: "<< sb->velocity << ", ";
        sb->velocity += dvdt * dt;
        std::cout << sb->position << std::endl;
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
