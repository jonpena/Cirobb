#ifndef SHAPES_H
#define SHAPES_H

#include <stdio.h>
#include "glut.h"
#include "RigidBody.h"

enum typeShape {circle, obb};

struct Shape
{
	//circle characteristics
	real radius; 
	//OBB characteristics
	Vec2 width;
	//Type of Shape
	typeShape type;
	//Rigid Body
	RigidBody* body;
	//Function Virtual To Draw Shape.
	virtual void DrawShape(void) = 0;
	//Function Virtual to Compute Mass And Inertia 
	virtual void CalculateMassInertia(const real&) = 0;
};

/*********************************** C I R C L E ****************************************/
/*********************************** C I R C L E ****************************************/

struct Circle : virtual public Shape
{	
	//Constructor Initialized
	Circle(Vec2 _X, real _radius, real _θ)
	{
		type = circle;
		radius = _radius;
		body = new RigidBody;
		body->X = _X;
		body->θ = _θ;
	}
	
	//Destructor By default
	~Circle(void) { delete body; }

	void DrawShape(void) override;
	
	void CalculateMassInertia(const real&) override;
};

/****************************** O B B ********************************/
/****************************** O B B ********************************/

struct OBB : virtual public Shape
{
	//Constructor Initialized
	OBB(Vec2 _X, Vec2 _width, real _θ)
	{
		type = obb;
		width = _width;
		body = new RigidBody;
		body->X = _X;
		body->θ = _θ;
	}

	//Destructor By default
	~OBB(void) { delete body; }

	void DrawShape(void) override;

	void CalculateMassInertia(const real&) override;
};

//Function To Draw A Point.
void DrawPoint(const Vec2&);

#endif