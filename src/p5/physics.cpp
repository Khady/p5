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

Derivative Physics::evaluate( const State &state, real_t dt, const Derivative &d ) const
{
//    Vector3 x = sphere->position + (d.dx * dt);
    Vector3 v = state.v + (d.dv * dt);

    Derivative output;
    output.dx = v;
    output.dv = state.force;

    return output;
}

State Physics::rk4(real_t dt, const State &state)
{
  Derivative a,b,c,d;
  a = evaluate( state, 0.0f, Derivative() );
  b = evaluate( state, dt*0.5f, a );
  c = evaluate( state, dt*0.5f, b );
  d = evaluate( state, dt, c );

  Vector3 dxdt = ( a.dx + 2.0f*(b.dx + c.dx) + d.dx ) * (1.0f / 6.0f);

  Vector3 dvdt =
    ( a.dv + 2.0f*(b.dv + c.dv) + d.dv ) * (1.0f / 6.0f);

  State s;
  s.x = dxdt * dt;
  s.v = dvdt * dt;
  return s;
}

void Physics::step( real_t dt )
{
    std::vector< SphereBody* >::iterator it;
    std::vector< SphereBody* >::iterator it2;
    std::vector< PlaneBody* >::iterator pt;
    std::vector< TriangleBody* >::iterator tt;

    for (it = spheres.begin(); it != spheres.end(); ++it)
      {
        SphereBody *sb = *it;

        for (tt = triangles.begin(); tt != triangles.end(); ++tt)
          {
            if (collides(*sb, **tt, collision_damping))
              {
                Vector3 n =
                  normalize(
                      cross(
                        (*tt)->vertices[0] - (*tt)->vertices[1],
                        (*tt)->vertices[1] - (*tt)->vertices[2]
                        )
                      );
                Vector3 u = sb->velocity - 2 * dot(sb->velocity, n) * n;
                sb->velocity = u - (collision_damping * u);
                if (squared_length(sb->velocity) <= 1.0f)
                  sb->velocity = Vector3::Zero();
              }
          }

        for (it2 = spheres.begin(); it2 != spheres.end(); ++it2)
          {
            if (collides(**it, **it2, collision_damping) && *it != *it2)
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
                if (squared_length(sb->velocity) <= 1.0f)
                  sb->velocity = Vector3::Zero();
                if (squared_length(sb2->velocity) <= 1.0f)
                  sb2->velocity = Vector3::Zero();
              }
          }

        for (pt = planes.begin(); pt != planes.end(); ++pt)
          {
            if (collides(*sb, **pt, collision_damping))
              {
                Vector3 u = sb->velocity - 2 * dot(sb->velocity, (*pt)->normal) * (*pt)->normal;
                sb->velocity = u - collision_damping * u;
                if (squared_length(sb->velocity) <= 1.0f)
                  sb->velocity = Vector3::Zero();
              }
          }


        sb->force = Vector3::Zero();
        for (SpringList::iterator si = springs.begin(); si != springs.end(); ++si) {
              (*si)->step(dt);
        }
        sb->apply_force(gravity, Vector3::Zero());

        // si pas de force et velocite, ne pas faire les tests
        State initMove = {sb->position, sb->velocity, sb->force / sb->mass};
        State s = rk4(dt, initMove);
        sb->position += s.x;
        sb->sphere->position = sb->position;
        sb->velocity += s.v;

        real_t i = (2.0f / 5.0f) * sb->mass * (sb->radius * sb->radius);
        Vector3 angular_accel = sb->torque / i;
        State initRot = {Vector3::Zero(), sb->angular_velocity, angular_accel};
        s = rk4(dt, initRot);
        if (s.x != Vector3::Zero())
          {
            Quaternion q(s.x, length(s.x));
            Quaternion qq = q * sb->orientation;
            sb->orientation = qq;
            sb->sphere->orientation = sb->orientation;
          }
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
