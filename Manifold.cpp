#include "Manifold.h"
#include "Collisions.h"


Manifold::Manifold(Shape* _A, Shape* _B) : numContacts(0)
{
	A = _A;
	B = _B;

	this->u = sqrtf(A->body->u * B->body->u); // friction

	Dispatcher[A->type][B->type](*this, A, B);
}

/****************************************************************************************************************
if the manifold exists in the list, we check if there is any contact point that can be started hot
This can be achieved with a certain distance heuristic, id, Etc. Erin Catto
*****************************************************************************************************************/
void Manifold::Update(Contact* newContacts, int numNewContacts)
{
	const real tolerance = 0.0001f;

	Contact mergedContacts[2];

	for (int i = 0; i < numNewContacts; i++)
	{
		int k = -1;
		
		for(int j = 0; k == -1 && j < numContacts; j++)
		{
			if((newContacts[i].position - contacts[j].position).SquareMagnitude() < tolerance) k = j; 
		}

		if (k > -1)
		{
			mergedContacts[i] = newContacts[i];
			mergedContacts[i].Pn = contacts[k].Pn;
			mergedContacts[i].Pt = contacts[k].Pt;
		}
		else 
			mergedContacts[i] = newContacts[i];
	}

	numContacts = numNewContacts;

	for (int i = 0; i < numContacts; i++) contacts[i] = mergedContacts[i];
}


/***********************************************************************************************************************
This PreStep method calculates the normal and tangent inverse effective mass of each contact point.

	A = J * M⁻¹ * Jt

Only in 2 dimensions is =

	             "J"                                    "M⁻¹"                 "Jt"

[nx, ny, n X r1, -nx, -ny, -n X r2] |m1⁻¹  0     0     0     0      0   | |nx     |
																	  |0		 m1⁻¹	 0     0     0		  0   | |ny     |
																	  |0     0     I1⁻¹  0     0      0   | |n X r1 |  
					                          |0     0     0     m2⁻¹  0      0   | |-nx    |  
					                          |0     0     0     0     m2⁻¹   0   | |-ny    |    
					                          |0     0     0     0     0      I2⁻¹| |-n X r2|

* This is only 1 constraint with 2 bodies involved.

* The final result will be a scalar that is equal to: A = m1⁻¹ + m2⁻¹ + (n X r1)^2 * I1⁻¹ + (n X r2)^2 * I2⁻¹

* The same goes for the At but with the tangent vector.

Equation = At = m1⁻¹ + m2⁻¹ + (t X r1)^2 * I1⁻¹ + (t X r2)^2 * I2⁻¹

Then we have the stabilization of baumgarte that transforms the position error into a speed error.
 
baumgarte = velocidad = x * ϵ / dt

After calculating the stabilization of baumgarte, use the same solver that corrects the speed to correct penetration.
*******************************************************************************************************************************************/
void Manifold::PreStep(const real& dt)
{
	const real k_biasFactor = 0.2f;
	const real k_allowedPenetration = 0.02f;

	RigidBody* b1 = A->body, * b2 = B->body;

	Vec2 tangent = Cross(normal, 1);

	for(int i = 0; i < numContacts; i++)
	{
		Contact* c = contacts + i;

		Vec2 r1 = c->position - A->body->X;
		Vec2 r2 = c->position - B->body->X;

		c->massNormal = b1->m⁻¹ + b2->m⁻¹  + pow2(Cross(normal, r1))  * b1->I⁻¹ + pow2(Cross(normal, r2))  * b2->I⁻¹;
		
		c->massNormal = 1.0f / c->massNormal;

		c->massTangent = b1->m⁻¹ + b2->m⁻¹ + pow2(Cross(tangent, r1)) * b1->I⁻¹ + pow2(Cross(tangent, r2)) * b2->I⁻¹;
		
		c->massTangent = 1.0f / c->massTangent;

		c->bias = max(0.0f, c->penetration - k_allowedPenetration) * k_biasFactor / dt;

		//Hot start
		Vec2 P = normal * c->Pn + tangent * c->Pt;

		b1->v -= P * b1->m⁻¹;
		b2->v += P * b2->m⁻¹;
		b1->ω -= Cross(P, r1) * b1->I⁻¹;
		b2->ω += Cross(P, r2) * b2->I⁻¹;			
	}
}




/****************************************************************************
 
 b = ϵ / Δt - J * V1; Relative Velocity 

 A = J * M⁻¹ * Jt;    inverse effective mass 

 λ =  P = x = -b * A⁻¹

 NO PENETRATION = λ = (v2 - v1) * n / M⁻¹ >= 0

 coulomb friction law = λt <= uλn.
*****************************************************************************/
void Manifold::ApplyImpulse(void)
{
	RigidBody* b1 = A->body, *b2 = B->body;

	Vec2 tangent = Cross(normal, 1.0f);

	for (int i = 0; i < numContacts; i++)
	{
		Contact* c = contacts + i; 

		Vec2 r1 = c->position - b1->X;
		Vec2 r2 = c->position - b2->X;

		Vec2 dv = b2->v + Cross(r2, b2->ω) - b1->v - Cross(r1, b1->ω);	

		real vn = dv * normal;
	
		real dPn = (-vn + c->bias) * c->massNormal; // Ax + b - c = 0 --> x = (-b + c) * A⁻¹; 
		
		real Pn0 = c->Pn;
		c->Pn = max(Pn0 + dPn, 0.0f);
		dPn = c->Pn - Pn0;

		real vt = dv * tangent;

		real dPt = -vt * c->massTangent; // Ax + b = 0 --> x = -b * A⁻¹; 

		real maxPt = c->Pn * u; // λt <= uλn
		real Pt0 = c->Pt;
		c->Pt = Clamp(-maxPt, maxPt, Pt0 + dPt);
		dPt = c->Pt - Pt0;		

		Vec2 P = normal * dPn + tangent * dPt;

		b1->v -= P * b1->m⁻¹;
		b2->v += P * b2->m⁻¹;
		b1->ω -= Cross(P, r1) * b1->I⁻¹;
		b2->ω += Cross(P, r2) * b2->I⁻¹;
	}
}