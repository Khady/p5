#include "p5/collisions.hpp"

namespace _462 {

bool collides( SphereBody& body1, SphereBody& body2, real_t collision_damping )
{
    if (
        dot(
            body1.velocity - body2.velocity,
            (body2.position - body1.position) / distance(body2.position, body1.position)
          ) >= 0
        && distance(body2.position, body1.position) < body1.radius + body2.radius
        )
      {
        return true;
      }
    return false;
}

Vector3 project_edge(const Vector3 &a, const Vector3 &b, const Vector3 &pp)
{
  return (
      (dot((pp - a), (b - a)) * (b - a))
      / squared_length(b - a)
      + a
      );
}

bool collides( SphereBody& body1, TriangleBody& body2, real_t collision_damping )
{
  Vector3 av = body2.vertices[0];
  Vector3 bv = body2.vertices[1];
  Vector3 cv = body2.vertices[2];

  Vector3 a = body1.position - body2.position;
  Vector3 n =
      normalize(
        cross(
          av - bv,
          bv - cv
          )
        );
  real_t d = dot(a, n);
  if (dot(n, body1.velocity) >= 0 && (abs(d) < body1.radius)) {
        return false;
  }
  Vector3 pp = body1.position - d * n;

  Vector3 ppp1 = project_edge(av, bv, pp);
  Vector3 ppp2 = project_edge(bv, cv, pp);
  Vector3 ppp3 = project_edge(av, cv, pp);

  if ((dot(av - bv, ppp1 - bv) >= 0 && dot(bv - av, ppp1 - av) >= 0)
      && (dot(bv - cv, ppp2 - cv) >= 0 && dot(cv - bv, ppp2 - bv) >= 0)
      && (dot(cv - av, ppp3 - av)>= 0 && dot(av - cv, ppp3 - cv) >= 0)
      && dot(n, body1.velocity) < 0 && (abs(d) < body1.radius))
    {
      return true;
    }

  return false;
}

bool collides( SphereBody& body1, PlaneBody& body2, real_t collision_damping )
{
  Vector3 a = body1.position - body2.position;
  real_t d = dot(a, body2.normal);
  return (dot(body2.normal, body1.velocity) < 0 && abs(d) < body1.radius);
}

}
