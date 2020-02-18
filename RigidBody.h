#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "CbMath.h"

struct RigidBody
{
	Vec2 X;   // Position
	Vec2 v;   // Linear velocity
	Vec2 F;   // Linear Force 
	real θ;   // Orientation or Angle
	real ω;   // Angular Velocity	
	real τ;   // Angular Force or Torque
	real m;   // Mass
	real m⁻¹; // Inverse Mass
	real I;   // Inertia
	real I⁻¹; // Inverse Inertia
	real u;   // Friction
	real d;   // Angular Damping
	
	//Set Dynamic Object.
	void Dynamic(void);

	//Set Static Object.
	void Static(void);
};
#endif