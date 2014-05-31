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
/*
void Spring::step( real_t dt )
{
    Vector3 f1 = -constant * (body1_offset) - damping * (body1->position / dt);
    body1->apply_force(f1, Vector3::Zero());

    Vector3 f2 = -constant * body2_offset - damping * (body2->position / dt);
    body2->apply_force(f2, Vector3::Zero());
}
*/
void Spring::step( real_t dt )
{
  Vector3 b1_attach = this->body1->position + this->body1_offset;
  Vector3 b2_attach = this->body2->position + this->body2_offset;

  real_t fLen = length(b1_attach - b2_attach);
  real_t x = fLen - this->equilibrium;

  Vector3 normDir = (b2_attach - b1_attach) / fLen;
  Vector3 velComp = normDir * x * dot(normalize(this->body1->velocity), normDir);

  Vector3 SpringForce = (normDir * x) * this->constant;// - this->damping * velComp;

  Vector3 Torque = cross( (b2_attach - this->body1->position), this->constant * ( (this->body2->position + this->body2_offset) - b2_attach) );

  this->body2->angular_velocity += Torque;


  if (squared_length(SpringForce) <= 1.0f)
    SpringForce = Vector3::Zero();

  this->body1->apply_force(SpringForce, Vector3::Zero());
//  this->body1->apply_force(Torque, this->body2_offset);
}


}
