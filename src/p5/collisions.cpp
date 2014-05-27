#include "p5/collisions.hpp"

namespace _462 {

bool collides( SphereBody& body1, SphereBody& body2, real_t collision_damping )
{
    if (distance(body1.position, body2.position) < body1.radius + body2.radius)
      {
        return true;
      }
    return false;
}

bool collides( SphereBody& body1, TriangleBody& body2, real_t collision_damping )
{
    // TODO detect collision. If there is one, update velocity

    return false;
}

bool collides( SphereBody& body1, PlaneBody& body2, real_t collision_damping )
{
    // TODO detect collision. If there is one, update velocity

	return false;
}

}
