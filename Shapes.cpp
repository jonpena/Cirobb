#include "Shapes.h"					

/**********************************************************************************************************************
*Although there are phyisics engine that add the masses manually to avoid certain problems such as: Low convergence of the solver
 when a heavy object is on top of a light object. It mass can also be proportional to the geometric area of the object.

* Circle area = Radius * PI * PI 

* Rectangle area = width * height 

These are the two areas that this physics engine uses to obtain the relative masses of the objects.

*There is also the inertia tensor: the inertia tensor reflects the mass distribution of a body or a system of rotating particles,
It can also be said that it is the resistance of the body to the rotation movement.

inertia matrix = |x  0  0|
	               |0  y  0|
	               |0  0  z|

2-dimensional physics engines have 3 degrees of freedom 2 of translation [X, Y, 0] and 1 of rotation [0, 0, Z]. It rotates on the Z axis.

2d inertia matrix = |0  0  0|
	                  |0  0  0|
	                  |0  0  z|

So when we multiply the matrix we will obtain a simple scalar value that represents the rotation on the Z axis.
It is also important to know that the moment of inertia depends only on the geometry of the body and the position of the axis of rotation.
Symmetric objects such as: A Circle or Rectangle have certain inertia tensors.

Equation: Inertia = mass * Distance * Distance

1- Circle Tensor =

Rx = Ry = Rz = Radius * Radius  

								|mRx*2/5    0          0|
								|								        |
	              |0       mRy*2/5       0|
								|								 		    |
	              |0          0    mRz*2/5|


2- Rectangle Tensor = 

Rx = y * y + z * z
Ry = x * x + z * z
Rz = x * x + y * y

						   |mRx/12		 0				  0|
							 |											 |
							 |0			  mRy/12		    0|
							 |											 |
							 |0          0     mRy/12|


In case 2d we only care about the value of the matrix on the Z axis. That is simply a scalar value.
***********************************************************************************************************************/
void Circle::CalculateMassInertia(const real& density)
{
	body->m = PI * radius * radius * density;
	body->invm = body->m ? 1.0f / body->m : 0.0f;
	body->I = body->m * radius * radius * 0.4f;
	body->invI = body->I ? 1.0f / body->I : 0.0f;
}



void OBB::CalculateMassInertia(const real& density)
{
	body->m = width.x * width.y * density;
	body->invm = body->m ? 1.0f / body->m : 0.0f;
	body->I = body->m * width.SquareMagnitude() / 12.0f; 
	body->invI = body->I ? 1.0f / body->I : 0.0f;
}



void Circle::DrawShape(void)
{
	real angle = body->orientation;
	glPushMatrix();
	glTranslatef(body->position.x, body->position.y, 0.0f);

	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_COLOR, GL_ONE_MINUS_SRC_ALPHA);
	glColor4f(0.4f, 0.4f, 0.4f, 0.4f);
	glBegin(GL_POLYGON);
	for(int i = 0; i <= 360; i++) // RADIAN [0, 2PI] --> DEGREE [0, 360]
	{
		glVertex2f(radius * cosf(i * RAD), radius * sinf(i * RAD));
	}
	glEnd();
	glDisable(GL_BLEND);

	glColor3f(0.65f, 0.65f, 0.65f);
	glBegin(GL_LINE_STRIP);
	for(int i = 0; i <= 360; i++) // RADIAN [0, 2PI] --> DEGREE [0, 360]
	{
		glVertex2f(radius * cosf(i * RAD), radius * sinf(i * RAD));
	}
	glEnd();
	glBegin(GL_LINES);
	glVertex2f(0.0f, 0.0f);
	glVertex2f(radius * cosf(angle), radius * sinf(angle));
	glEnd();
	glColor3f(1.0f, 1.0f, 1.0f);
	glPopMatrix();
}



void OBB::DrawShape(void)
{
	glPushMatrix();
	glTranslatef(body->position.x, body->position.y, 0.0f);

	Mat2 Rot(body->orientation);
	Vec2 p1 = Rot * width * 0.5f;
	Vec2 p2 = Rot * Vec2(width.x, -width.y) * 0.5f;	

	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_COLOR, GL_ONE_MINUS_SRC_ALPHA);
	glColor4f(0.4f, 0.4f, 0.4f, 0.4f);
	glBegin(GL_POLYGON);
	glVertex2f( p1.x, p1.y);
	glVertex2f( p2.x, p2.y);
	glVertex2f(-p1.x,-p1.y);
	glVertex2f(-p2.x,-p2.y);
	glEnd();
	glDisable(GL_BLEND);

	glColor3f(0.65f, 0.65f, 0.65f);
	glBegin(GL_LINE_LOOP);
	glVertex2f( p1.x, p1.y);
	glVertex2f( p2.x, p2.y);
	glVertex2f(-p1.x,-p1.y);
	glVertex2f(-p2.x,-p2.y);
	glEnd();
	glColor3f(1.0f, 1.0f, 1.0f);
	glPopMatrix();
}



void DrawPoint(const Vec2& p, const int& cases, const int& size)
{
	glPointSize(size);	
	if(cases == 0) glColor3f(1.0f, 0.0, 0.0f);
	if(cases == 1) glColor3f(0.0f, 1.0, 0.0f);
	if(cases == 2) glColor3f(0.0f, 0.0, 1.0f);
	glBegin(GL_POINTS);
	glVertex2f(p.x, p.y);
	glEnd();
	glColor3f(1.0f, 1.0f, 1.0f);
}



void DrawLine(const Vec2& p1, const Vec2& p2)
{
	glLineWidth(1.0);
	glColor3f(1.0f, 0.0, 0.0f);
	glBegin(GL_LINES);
	glVertex2f(p1.x, p1.y);
	glVertex2f(p2.x, p2.y);
	glEnd();
	glColor3f(1.0f, 1.0f, 1.0f);
	glLineWidth(1);
}