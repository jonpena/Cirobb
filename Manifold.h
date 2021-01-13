
#ifndef MANIFOLD_H
#define MANIFOLD_H

#include "RigidBody.h"
#include "Shapes.h"
#include <map>



struct Contact
{
	Contact(void) : Pn(0.0f), Pt(0.0f), bias(0.0f), restitution(0.0f), Oldpoint(FLT_MAX, FLT_MAX) {;}

	Vec2 position; // Position of Contact Point
	Vec2 Oldpoint;  // Position of Contact Old Point
	real penetration; // Penetration depth
	real Pn; //Sum of accumulated projected normal impulse.
	real Pt; //Sum of accumulated projected tangent impulse.
	real massNormal, massTangent; //Normal and tangent Effective Mass
	real bias; //Baumgarte Stabilization
	real restitution; // Restitution to Bounce
};


struct PostPosition
{
	real orientationOldA; 
	real orientationOldB;
	Vec2 point1[2];
	Vec2 point2[2];

	PostPosition(void) : orientationOldA(0.0f), orientationOldB(0.0f) {;}
};



struct ManifoldKey
{
	RigidBody* A, *B;

	ManifoldKey(RigidBody* _A, RigidBody* _B) : A(_A), B(_B) {;}
};



struct Manifold
{
	enum {MAX_POINTS = 2};

	int numContacts;

	Contact contacts[MAX_POINTS];

	PostPosition postPosition;

	real u; //Friction mixed
	
	real e; //Restitution Max

	RigidBody* A; RigidBody* B;

	Vec2 normal; // Normal

	Manifold(RigidBody*, RigidBody*);

	void Update(Contact contacts[MAX_POINTS], const int&);

	void PreStep(const real&);

	void WarmStarting(void);

	void ApplyImpulse(void);

	void ApplyCorrection(void);

	real RecalculatePenetration(Vec2& normal, Vec2& contactPoint, const int& i);
};



// This is used by std::set. Erin catto
inline bool operator < (const ManifoldKey& a1, const ManifoldKey& a2)
{
	return (a1.A < a2.A || a1.A == a2.A && a1.B < a2.B);
}

#endif