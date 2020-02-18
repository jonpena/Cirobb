#include "RigidBody.h"

//Method that sets rigid body values to dynamic
void RigidBody::Dynamic(void)
{
	this->u = 0.3f; // range[0..1]
	this->v.SetZero();
	this->F.SetZero();
	this->ω = 0.0f;
	this->τ = 0.0f;
	this->d = 0.1f; // range[0..1];
}


//Method that sets rigid body values to static
void RigidBody::Static(void)
{
	this->u = 1.0f; // range[0..1]
	this->ω = 0.0f;
	this->τ = 0.0f;
	this->d = 0.0f; // range[0..1]
	this->v.SetZero();
	this->F.SetZero();
}