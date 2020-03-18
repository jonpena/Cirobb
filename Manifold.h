#ifndef MANIFOLD_H
#define MANIFOLD_H

#include "Shapes.h"

struct Contact
{
	Contact() : Pn(0.0f), Pt(0.0f) {;}

	Vec2 position;
	real penetration;
	real Pn; //Sum of accumulated projected normal velocity error.
	real Pt; //Sum of accumulated projected tangent velocity error
	real massNormal, massTangent; //Normal and tangent effective mass
	real bias; //stabilization of baumgarte
};


struct ManifoldKey
{
	Shape* A; Shape* B;

	ManifoldKey(Shape* _A, Shape* _B) : A(_A), B(_B) {;}
};


struct Manifold
{
	enum {MAX_POINTS = 2};

	int numContacts;

	Contact contacts[MAX_POINTS];

	real u; //friction

	Vec2 normal;

	Shape* A; Shape* B;

	Manifold(Shape* _A, Shape* _B);

	void Update(Contact contacts[MAX_POINTS], int);

	void PreStep(const real&);

	void ApplyImpulse();
};


// This is used by std::set. Erin catto
inline bool operator < (const ManifoldKey& a1, const ManifoldKey& a2)
{
	return (a1.A == a2.A && a1.B < a2.B || a1.A < a2.A);
}

#endif
