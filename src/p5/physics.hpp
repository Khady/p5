#ifndef _462_PHYSICS_PHYSICS_HPP_
#define _462_PHYSICS_PHYSICS_HPP_

#include "math/math.hpp"
#include "math/quaternion.hpp"
#include "math/vector.hpp"
#include "p5/body.hpp"
#include "p5/spherebody.hpp"
#include "p5/trianglebody.hpp"
#include "p5/planebody.hpp"
#include "p5/spring.hpp"
#include "p5/collisions.hpp"

#include <vector>

namespace _462 {

typedef struct Derivative
{
    Vector3 dx;
    Vector3 dv;
} Derivative;

typedef struct State
{
  Vector3 x;
  Vector3 v;
  Vector3 force;
} State;

class Physics
{
public:
    Vector3 gravity;
    real_t collision_damping;

    Physics();
    ~Physics();

    void step( real_t dt );
    void add_sphere( SphereBody* s );
    size_t num_spheres() const;
    void add_plane( PlaneBody* p );
    size_t num_planes() const;
    void add_triangle( TriangleBody* t );
    size_t num_triangles() const;
    void add_spring( Spring* s );
    size_t num_springs() const;
    Derivative evaluate( const State &state, real_t dt, const Derivative &d ) const;
    State rk4(real_t dt, const State &state);

    void reset();

private:
    typedef std::vector< Spring* > SpringList;
    typedef std::vector< SphereBody* > SphereList;
    typedef std::vector< PlaneBody* > PlaneList;
    typedef std::vector< TriangleBody* > TriangleList;

    SpringList springs;
    SphereList spheres;
    PlaneList planes;
    TriangleList triangles;
};

}

#endif

