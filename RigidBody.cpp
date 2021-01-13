#include "RigidBody.h"
#include "Shapes.h"


RigidBody::RigidBody(Shape& shape, const Vec2& _position, const real& _orientation)
{
	position = _position; 
	orientation = _orientation;
	this->shape = shape.newShape();
	this->shape->body = this;
}


//Method that sets rigid body values to dynamic
void RigidBody::Dynamic(const real& density)
{
	this->angularVelocity = 0.0f;
	this->torque = 0.0f;
	this->velocity.SetZero();
	this->force.SetZero();
	this->friction = 0.3f;       // Range[0..1]  
	this->restitution = 0.3f;   // Range[0..1] 
	this->angularDamping = 0.0f; // Range[0..1]
	this->linearDamping = 0.0f;  // Range[0..1]
	this->gravityScale = 1.0f;   // Range[0..1]
	this->shape->CalculateMassInertia(density);
}


//Method that sets rigid body values to static
void RigidBody::Static(void)
{
	this->angularVelocity = 0.0f;
	this->torque = 0.0f;
	this->velocity.SetZero();
	this->force.SetZero();
	this->friction = 1.0f;       //Range[0..1]
	this->restitution = 0.0f;    //Range[0..1] 
	this->angularDamping = 0.0f; //Range[0..1]
	this->linearDamping  = 0.0f; //Range[0..1]
	this->gravityScale   = 0.0f; //Range[0..1]
	this->shape->CalculateMassInertia(0);
}