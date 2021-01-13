#ifndef SHAPES_H
#define SHAPES_H

#include <stdio.h>
#include <iostream>
#include "glut.h"
#include "RigidBody.h"


enum typeShape {circle, obb};


struct Shape
{
	//Circle characteristics
	real radius; 
	//OBB characteristics
	Vec2 width;
	//Type of Shape
	typeShape type;
	//Rigid Body
	RigidBody* body;
	//Function Virtual To Draw Shape.
	virtual void DrawShape(void) = 0;
	//Function Virtual To create Dynamic Shape
	virtual Shape* newShape(void) = 0;
	//Function Virtual to Compute Mass And Inertia 
	virtual void CalculateMassInertia(const real&) = 0;
};

/*********************************** C I R C L E ****************************************/
/*********************************** C I R C L E ****************************************/

struct Circle : virtual public Shape
{	
	//Constructor Initialized
	Circle(const real& _radius)
	{
		radius = _radius; type = circle;  
	}
	
	//Destructor By default
	~Circle(void) {;}

	Shape* newShape() override {return new Circle(this->radius);}

	void DrawShape(void) override;
	
	void CalculateMassInertia(const real&) override;
};


/****************************** O B B ********************************/
/****************************** O B B ********************************/


struct OBB : virtual public Shape
{
	//Constructor Initialized with Vector
	OBB(const Vec2& _width) 
	{
		type = obb; width = _width;
	}

	//Constructor Initialized with scalars
	OBB(const real& x, const real& y)
	{
		type = obb; width.Set(x, y);
	}

	//Destructor By default
	~OBB(void) {;}

	Shape* newShape(void) override {return new OBB(this->width);}

	void DrawShape(void) override;

	void CalculateMassInertia(const real&) override;
};

//Function To Draw A Point: Point, Color, Size.
void DrawPoint(const Vec2& p, const int&, const int&);

//Fucntion To Draw A Line
void DrawLine(const Vec2& p1, const Vec2& p2);

#endif