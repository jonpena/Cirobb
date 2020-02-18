#include "Shapes.h"					

/**********************************************************************************************************************
**Although there are motors that add the masses manually to reduce certain problems such as: low convergence of the solver
  when a heavy object is on top of a light object. It can also be proportional to the geometric area of the object for example:

* Circle Area = Radius * PI * PI 

* Rectangle Area = width * height 

* Rectangle Perimetro = width * height * 4


These are the two areas that this physics engine uses to obtain the relative mass of the object.

**There is also the inertia tensor: the inertia tensor reflects the mass distribution of a body or a system of rotating particles,
It can also be said that it is the resistance of the object to the rotation movement.

inertia matrix = |x  0  0|
	               |0  y  0|
	               |0  0  z|

2-dimensional physics engines have 3 degrees of freedom 2 of translation [X, Y, 0] and 1 of rotation [0, 0, Z]. It rotates on the Z axis.

2d inertia matrix = |0  0  0|
	                  |0  0  0|
	                  |0  0  z|

So when we multiply the Matrix we will obtain a simple scalar value that represents the rotation on the Z axis.
It is also important to know that the moment of inertia depends only on the geometry of the body and the position of the axis of rotation.
Symmetric objects such as a circle or rectangle have certain moments of inertia.

Equation: I = mass * Distance * Distance

1- Circle Tensor =

Rx = Ry = Rz = Radius * Radius  

								|mRx*2/5    0          0|
								|								        |
	              |0       mRy*2/5       0|
								|								 		    |
	              |0          0    mRz*2/5|


2- Rectangle Tensor = 

Rx = y*y + z*z
Ry = x*x + z*z
Rz = x*x + y*y

						   |mRx/12		 0				  0|
							 |											 |
							 |0			  mRy/12		    0|
							 |											 |
							 |0          0     mRy/12|


In case 2d we only care about the value of the matrix on the Z axis. That is simply a scalar value.
***********************************************************************************************************************/




void Circle::CalculateMassInertia(const real& density)
{
	if(!density)
	{
		body->Static();
		body->m = body->m⁻¹ = 0.0f;
		body->I = body->I⁻¹ = 0.0f;
	}
	else
	{
		body->Dynamic();
		body->m = PI * radius * radius *  density;
		body->m⁻¹ = (body->m)? 1.0f / body->m : 0.0f;
		body->I = body->m * radius * radius * 0.4f;
		body->I⁻¹ = (body->I)? 1.0f / body->I : 0.0f;
	}
}




void OBB::CalculateMassInertia(const real& density)
{
	if(!density)
	{
		body->Static();
		body->m = body->m⁻¹ = 0.0f;
		body->I = body->I⁻¹ = 0.0f;
	}
	else
	{
		body->Dynamic();
		body->m = width.x * width.y * 4.0f * density;
		body->m⁻¹ = (body->m)? 1.0f / body->m : 0.0f;
		body->I = body->m * width.SquareMagnitude() / 3.0f;  // 4 / 12 = 1 / 3 
		body->I⁻¹ = (body->I)? 1.0f / body->I : 0.0f;
	}
}





void Circle::DrawShape(void)
{
	glPushMatrix();
	glTranslatef(body->X.x, body->X.y, 0.0f);
	glBegin(GL_LINE_STRIP);
	for(int i = 0; i <= 360; i++) // RADIAN [0, 2 * PI] --> DEGREE [0, 360]
	{
		glVertex2f(radius * cosf(i * RAD), radius * sinf(i * RAD));
	}
	glEnd();
	glBegin(GL_LINES);
	glVertex2f(0.0f, 0.0f);
	glVertex2f(radius * cosf(body->θ), radius * -sinf(body->θ));
	glEnd();
	glPopMatrix();
}



void OBB::DrawShape(void)
{
	glPushMatrix();
	glTranslatef(body->X.x, body->X.y, 0.0f);
	Mat2 matriz(body->θ);
	Vec2 rotation1 = matriz * width;
	Vec2 rotation2 = matriz * Vec2(width.x, -width.y);	
	glBegin(GL_LINE_LOOP);
	glVertex2f(-rotation1.x,-rotation1.y);
	glVertex2f(rotation2.x,  rotation2.y);
	glVertex2f(rotation1.x,  rotation1.y);
	glVertex2f(-rotation2.x,-rotation2.y);
	glEnd();
	glPopMatrix();
}


void DrawPoint(const Vec2& p)
{
	glPointSize(5.0f);
	glColor3f(0.0f, 0.8, 0.0f);
	glBegin(GL_POINTS);
	glVertex2f(p.x, p.y);
	glEnd();
	glColor3f(1.0f, 1.0f, 1.0f);
}
