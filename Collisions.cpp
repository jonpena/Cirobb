#include "Collisions.h"


/************************************************************************************************
Collision detection is one of the most important and problematic parts of physics engines.
Collision detection gives us information about: normal, penetration, Contact point, ETC.
There are three stages of collision detection: broad phase, medium phase and narrow phase.
The collision detection found in this file.cpp is only narrow phase.
The broad phase is a brute force algorithm of complexity O (n ^ 2). and has no middle phase.
I should also clarify that these collision detection methods were only tested in 2d.

Thank you very much Dirk Gregorius.
*************************************************************************************************/









/************************************************************************************************
Collision Detection Circle to Circle is one of the fastest and easiest. All this due to its properties.
The circle is a highly symmetrical form: each line through the center forms a line of reflection symmetry
and has rotational symmetry around the center for each angle. The circumference and radius of a circle are proportional.
This means that if there is a point on the circumference and the origin is the center of the circle then Radius = sqrt (x * x + y * y).
taking advantage of the property that the circles are highly symmetrical We can obtain the local and global minimum distance
subtracting the centers. then if (|c2 - c1| <= r1 + r2)) then there is a collision and we have to extract this information for the solver.
When a circle collides with any convex polygon, a single point of contact is sufficient.

* Penetration = r1 + r2 - |c2 - c1|

* Normal = (c2 - c1) / |c2 - c1|

* t = (r1 - r2 + |c2 - c1|) * 0.5f

* ContactPoint  = c1 + n * t  --> The point in the middle. This is a way of doing it.

There is a degenerate case and it happens when the centers of the circles coincide resulting in |c2 - c1| = 0. This case must be manipulated manually.
*************************************************************************************************/
void CircleToCircle(Manifold& m, Shape* c1, Shape* c2)
{
	//Distance from center to center
	Vec2 distance = c2->body->X - c1->body->X;
	
	//Suma de Radii
	real radius = c2->radius + c1->radius;

	//Squared Magnitude |c2 - c1|^2
	real magnitude = distance.SquareMagnitude();

	//Since we are only interested in knowing if there is a collision, we can avoid the expensive square root by raising the two members of the equation to power 2.
	if(magnitude > radius * radius) return;
	
	//Magnitude |c2 - c1|
	magnitude = sqrtf(magnitude);

	m.numContacts = 1;
	m.contacts[0].penetration = radius - magnitude;
	m.normal = (magnitude)? distance * (real(1) / magnitude) : Vec2(0, -1);
	m.contacts[0].position = c1->body->X + m.normal * ((c1->radius - c2->radius + magnitude) * 0.5f);
}





/************************************************************************************************
Collision detection OBB Circle (Oriented bounding box) Can be simplified to a collision detection
Circle to AABB (bounding box aligned with the axis) by moving the OBB to its local space Transforming it into an AABB (bounding box aligned with the axis).
Then we have to find the point closest to the circle. After obtaining the closest point we have a problem of the form | c2 - c1 | <= r1 + r2.
But since a point has radius = 0 the problem is simplified to | c2 - c1 | <= r1.
if there is a collision we have to extract our information for the solver and then return it to the world space. Just one point of contact is enough.

There are two cases:

Shallow Penetration: The center of the circle is outside the OBB.

Deep    Penetration: The center of the circle is inside the OBB.

Case = |c2 - c1| == 0 ? Deep Penetration : Shallow Penetration

if(We have a case of Shallow Penetration)then We have all the necessary information to calculate: normal, penetration, contact point.

if(We have a case of Deep Penetration) then we have to find the axis of minimum penetration to calculate: normal, penetration, Contact.

*************************************************************************************************/
void CircleToOBB(Manifold& m, Shape* c1, Shape*	b1)
{
	//Rotation matrix
	Mat2 Rot(-b1->body->θ);
	//Circle in the local AABB space
	Vec2 localSpace = Rot * (c1->body->X - b1->body->X);
	//Closest point of the AABB to Circle.
	Vec2 closest = localSpace;

	//4 Conditions to find the closest point of the circle.
	if(closest.x < -b1->width.x) closest.x = -b1->width.x;
	if(closest.x >  b1->width.x) closest.x =	b1->width.x;
	if(closest.y < -b1->width.y) closest.y = -b1->width.y;
	if(closest.y >  b1->width.y) closest.y =  b1->width.y;

	//Distance from the point closest to the center of the circle
	Vec2 distance = closest - localSpace;

	//Squared magnitude |c2 - c1|^2
	real magnitude = distance.SquareMagnitude();

	//Since we are only interested in knowing if there is a collision, we can avoid the expensive square root by raising the two members of the equation to power 2.
	if(magnitude > c1->radius * c1->radius) return;
 	
	//Case Shallow Penetration.
	if(magnitude)
	{
		magnitude = sqrtf(magnitude);
		m.normal = distance * (real(1) / magnitude);		
	}
	else //Case Deep Penetration. 
	{
		distance = closest.GetAbs() - b1->width;
		magnitude = max(distance.x, distance.y);
		m.normal = (distance.x > distance.y)? Vec2((closest.x < 0)? 1 : -1, 0) : Vec2(0, (closest.y < 0)? 1 : -1);
	}

	m.numContacts = 1;
	m.contacts[0].penetration = c1->radius - magnitude;
	m.normal = Rot.Transpose(m.normal); //return to space world
	m.contacts[0].position = c1->body->X + m.normal * magnitude;
}




//Collision Detection Box Oriented To Circle
void OBBToCircle(Manifold& m, Shape* b1, Shape* c1)
{
	CircleToOBB(m, c1, b1);  m.normal = -m.normal;
}






/*******************************************************************************************
Collision Detection from OBB to OBB It is the most difficult method we will see in this file.cpp.
This method uses the SAT (separation axis theorem), finding the minimum and maximum to obtain the minimum penetration axis.
Collision detection from OBB to OBB also has a complete set of problems, such as:
Consistent points, flip-flop characteristics, coplanar edges, rotation errors, ETC.
OBBs (oriented delimitation tables) have certain optimizations due to their simplicity. Although there are 8 normals that must be tested, 
each OBB has 2 parallel edges that reduce it to 4 normals that must be tested.
We can also transform the collision detection problem into an AABB TO OBB test by simplifying the problem.
I must clarify that this code is not optimized and simplified to facilitate the understanding of the code. 
I must also clarify that the method written in this file is not completely stable.
**********************************************************************************************/


//this function returns index of The incident edge, The incident edge is simply the most anti-parallel edge on the other OBB.
inline int FindIncidentEdge(const Vec2& normal, Vec2 axis1, Vec2 axis2)
{
	real projection1 = axis1 * normal;
	real projection2 = axis2 * normal;

	int i = 0;

	if(projection1 > 0.0f) {projection1 *= -1; i = 2;}

	if(projection1 >  projection2) return 1;
	if(projection1 > -projection2) return 3;

	return i;
}


//This function clip The incident edge with the reference edge.
inline Vec2 ClipSegmentToLine(Vec2& I1, Vec2& I2, Vec2& R1, const Vec2& n)
{
	Vec2 d1 = I2 - I1;
	Vec2 d2 = R1 - I1;

	real t = (d2.y * n.x - d2.x * n.y) / (d1.y * n.x - d1.x * n.y);

	return (0 > t || t > 1)? I1 : I1 + d1 * t;
}


void OBBToOBB(Manifold& m, Shape* b1, Shape* b2)
{
	//Rotation matrix
	Mat2 Rot(b2->body->θ - b1->body->θ);
	
	//these are the 4 normal axes that must be tested
	Vec2 axes[4] = {Vec2(-1, 0), Vec2(0, -1), -Rot.Column0(), -Rot.Column1()};
	
	//These are the widths of object b2 in the local space of b1
	Vec2 widthB1 = Rot * b2->width;
	Vec2 widthB2 = Rot * Vec2(-b2->width.x, b2->width.y);	

	//Local Space of b1
	Vec2 localSpace = Rot.Rotate(-b1->body->θ, b2->body->X - b1->body->X);
	
	//the 4 vertices of b1 in your local space. The normal ones of b1 and the edges must have coherence.
	Vec2 vertsA[4] = {Vec2(-b1->width.x, b1->width.y), -b1->width, Vec2(b1->width.x, -b1->width.y), b1->width};
	
	//the 4 vertices of b2 in the local space of b1. The normal ones of b2 and the edges must have coherence.
	Vec2 vertsB[4] = {localSpace + widthB2, localSpace - widthB1, localSpace - widthB2,  localSpace + widthB1};

	/** We do not need to make a function to find the min and max on b1 and b2. **/

	// * b1 axes[0] : min = -r1->width.x  and  max = r1->width.x 
	// * b1 axes[1] : min = -r1->width.y  and  max = r1->width.y 
	// * b1 axes[2] : value = max(abs(vertsA[0] * axes[2]), abs(vertsA[1] * axes[2]));
	//   min = -value  and  max = value
	// * b1 axes[3] : value = max(abs(vertsA[0] * axes[3]), abs(vertsA[1] * axes[3]));
	//   min = -value and max = value

	// * b2 axes[0] : value =  max(abs(widthB1.x), abs(widthB2.x))
	//   min = -value  and  max = value
	// * b2 axes[1] : value =  max(abs(widthB1.y), abs(widthB2.y))
	//   min = -value  and  max = value
	// * b2 axes[2] : min = vertsB[3] * axes[2]  and  max = vertsB[1] * axes[2]
	// * b2 axes[3] : min = vertsB[3] * axes[3]  and  max = vertsB[1] * axes[3]

	// if(b2.min <= 0 && b1.min <= b2.max) then hay colision.

	real Ax = max(absf(axes[2] * vertsA[0]), absf(axes[2] * vertsA[1]));
	real Ay = max(absf(axes[3] * vertsA[0]), absf(axes[3] * vertsA[1]));

	real Bx = max(absf(widthB1.x), absf(widthB2.x)) - vertsA[1].x;
	real By = max(absf(widthB1.y), absf(widthB2.y)) - vertsA[1].y;

	// b2.min - b1.max
	real v1[4] = {Bx + localSpace.x, By + localSpace.y, Ax - axes[2] * vertsB[3], Ay - axes[3] * vertsB[3]};

	// b1.min - b2.max
	real v2[4] = {Bx - localSpace.x, By - localSpace.y, Ax + axes[2] * vertsB[1], Ay + axes[3] * vertsB[1]};

	//if (any of these 4 values of v1 is < 0) then we find an axis of separation. There is no collision.
	//if (any of these 4 values of v2 is < 0) then we find an axis of separation. There is no collision.

	int minimumAxis = 0; 
	real penetration = min(v1[0], v2[0]); 

	if(penetration < 0.0f) return;

	const real relativeTol = 0.95f;
	const real absoluteTol = 0.01f;

	//We test the 3 missing axes
	for(int i = 1; i < 4; i++)
	{
		real minimum = min(v1[i], v2[i]);

		if(minimum < 0.0f) return; // if(minimum < 0) then We find an axis of separation.

		// we stored the index of The axis of minimum penetration
		if(minimum * relativeTol + penetration * absoluteTol < penetration)
		{
			minimumAxis = i; penetration = minimum;
		}
	}

	int negation = v1[minimumAxis] < v2[minimumAxis] ? 1 : -1;

	//if(minimumAxis < 2) then Change the direction of the normal axis.
	int flip = (minimumAxis <  2)? -1 : 1;
	
	int side = (negation == flip)?  2 : 0;

	Vec2 ref[2], inc[2];

	Vec2 normalLocal = axes[minimumAxis] * -negation;
	
	//The reference edge is in b1.
	if(flip == -1)
	{
		normalLocal = -normalLocal;
		ref[0] = vertsA[minimumAxis + side];
		ref[1] = (minimumAxis + side == 3)? vertsA[0] : vertsA[minimumAxis + side + 1];
		int i = FindIncidentEdge(-normalLocal, axes[2], axes[3]);
		inc[0] = vertsB[i];
		inc[1] = (i == 3)? vertsB[0] : vertsB[i + 1];
	}
	//The reference edge is in b2.
	else
	{
		minimumAxis -= 2;
		ref[0] = vertsB[minimumAxis + side];
		ref[1] = (minimumAxis + side == 3)? vertsB[0] : vertsB[minimumAxis + side + 1];
		int i = FindIncidentEdge( normalLocal, axes[0], axes[1]);
		inc[0] = vertsA[i];
		inc[1] = (i == 3)? vertsA[0] : vertsA[i + 1];
	}

	int numContacts = 0;

	//finding the contact points.
	for(int i = 0; i < 2; i++)
	{
		Vec2 pClipping = ClipSegmentToLine(inc[1 - i], inc[i], ref[i], normalLocal);

		penetration = normalLocal * (ref[0] - pClipping); //(p1 - p2) * n 

		if(penetration > 0.0f)
		{
			m.contacts[numContacts].penetration = penetration;
			m.contacts[numContacts].position = Rot.Transpose(pClipping + normalLocal * penetration) + b1->body->X;
			numContacts++;
		}
	}
	m.numContacts = numContacts;
	m.normal = Rot.Transpose(-normalLocal * flip);
}
