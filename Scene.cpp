#include "Scene.h"

typedef std::pair<ManifoldKey, Manifold> ManPair;
typedef std::map<ManifoldKey, Manifold>::iterator ManifoldAux;


//Add a Shape to the list
void Scene::Add(Shape* body)
{
	bodies.push_back(body);
}

//Clear the list and map
void Scene::Clear(void)
{
	bodies.clear(); manifolds.clear();
}

 //Contact Persistence Algorithm from erin catto.
void Scene::BroadPhase()
{
	for(int i = 0; i < (int)bodies.size(); i++)
	{
	  Shape* bi = bodies[i];

		for(int j = i + 1; j < (int)bodies.size(); j++)
		{
			Shape* bj = bodies[j];

			if(bi->body->m + bj->body->m == 0.0f) continue;

			ManifoldKey key(bi, bj);
			Manifold newMan(bi, bj);
		
			if(newMan.numContacts > 0)
			{
				ManifoldAux iter = manifolds.find(key);

				if(iter == manifolds.end())
				{
					manifolds.insert(ManPair(key, newMan));
				}
				else
				{
					iter->second.normal = newMan.normal;
					iter->second.Update(newMan.contacts, newMan.numContacts);
				}
			}
			else 
				manifolds.erase(key);
		}
	}
}



/**********************************************************************************************
There are many ways to integrate the kinematic equations of motion. But the semi-implicit (symplectic) Euler
It is the most widely used method in real-time physics engines due to its acceptable stability and speed.


Semi-Implicit (Symplectic) Euler

Linear movement equations

v = v0 + P * M⁻¹ + Fext * M⁻¹ * Δt

x = x0 + v * Δt

Rotational movement equations

ω = ω0 + L * I⁻¹ + τext * I⁻¹ * Δt

θ = θ0 + ω * Δt
***********************************************************************************************/



void Scene::Step(const real& dt)
{
	BroadPhase();

	for(auto temp : bodies)
	{
		RigidBody* b = temp->body;

		if(b->m <= 0.0f) continue;

		b->v += (b->F * b->m⁻¹ + gravity) * dt; // v = v0 + Fext * M⁻¹ * Δt
		b->ω += b->τ * b->I⁻¹ * dt;	            // ω = ω0 + τext * I⁻¹ * Δt	
		b->ω *= pow(0.97, b->d); // min = 0.97 max = 1.0;
	}


	for(ManifoldAux temp = manifolds.begin(); temp != manifolds.end(); temp++)
	{
		temp->second.PreStep(dt);
	}


	for(int i = 0; i < iterations; i++)
	{
		for(ManifoldAux temp = manifolds.begin(); temp != manifolds.end(); temp++)
		{
			temp->second.ApplyImpulse(); // v = v0 + P * M⁻¹   and   ω = ω0 + L * I⁻¹;
		}
	}

	for(auto temp : bodies)
	{
		RigidBody* b = temp->body;
		
		b->X += b->v * dt;  // x = x0 + v * Δt
		b->θ += b->ω * dt;  // θ = θ0 + ω * Δt 
		b->F.SetZero(); 
		b->τ = 0.0f;
	}
}
