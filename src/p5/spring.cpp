#include "math/math.hpp"
#include "math/vector.hpp"
#include "math/quaternion.hpp"
#include "math/matrix.hpp"
#include "p5/spring.hpp"
#include "p5/body.hpp"
#include "p5/spherebody.hpp"
#include <iostream>

namespace _462 {

Spring::Spring()
{
    body1_offset = Vector3::Zero();
    body2_offset = Vector3::Zero();
    damping = 0.0;
}

void Spring::step( real_t dt )
{
    Vector3 f1 = -constant * (body1_offset) - damping * (body1->position / dt);
    body1->apply_force(f1, Vector3::Zero());

    Vector3 f2 = -constant * body2_offset - damping * (body2->position / dt);
    body2->apply_force(f2, Vector3::Zero());
}

}
