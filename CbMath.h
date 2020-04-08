#ifndef CBMATH_H
#define CBMATH_H

#include <math.h>
#include <cfloat>

typedef float real;
//Irrational Constant PI
const real PI = 3.14159265358979323846264f;
//Conversion of Degrees to Radians
const real RAD = PI / 180;
//Conversion of Radians to Radians
const real DEG = 180 / PI;
//Define functions if there is no other declaration.
#if !defined(min) && !defined(max)
//the minimum value returns
#define min(a,b) ((a > b)? (b) : (a))
//the maximum value returns
#define max(a,b) ((a < b)? (b) : (a))
#endif
//The Absolute Value Returns
#define absf(a) ((a < 0)? -(real)(a) : (a))
//The Scalar returns to the Square
#define pow2(a) ((a) * (a))


/********************************************************************************************
A tuple of n real numbers called components of the vector is called a vector of n dimension.
Vectors allow us to represent physical vector quantities.
The abstraction of vectors is very important to understand. instead of solving each incognita separately
vectors can abstract the dimensions x, y, z ... Dn so that they can be resolved at once.
There are two ways to represent vector:

Polar coordinates: magnitude, direction and sense;

Cartesian coordinates: The components of the vector in the Cartesian axes[x, y, z,...n].
 
 This project makes extensive use of vectors so it is important to know what each operation and representation does.
********************************************************************************************/

struct Vec2
{
	real x, y; 
	
	//Constructor By Default
	Vec2() : x(0.0f), y(0.0f) {;}

	//Contructor Alternative
	Vec2(real _x, real _y) : x(_x), y(_y) {;}
	
	//Set Values In The Vector
	void Set(const real& _x, const real& _y)
	{
		x = _x;
		y = _y;
	}

	//Vector Magnitude --> Pythagoras theorem |C| = sqrt(A^2 + B^2)
	real Magnitude(void)
	{
		return sqrtf(x * x + y * y);
	}
	
	//Vector Squared Magnitude --> Pythagoras theorem C^2 = A^2 + B^2
	real SquareMagnitude(void)  
	{
		return x * x + y * y;
	}
	
	/*************************************************************************************
	The normalization of the vector is a very useful operation gives us the unit direction of the vector.

	cos(x) = adyacent / hypotenuse  -->  x / magnitude 
	sin(x) = opposite / hypotenusa  -->  y / magnitude
	
	*************************************************************************************/
	Vec2 normalize(void)
	{
		real mag = Magnitude();

		return (mag)? *this * (1.0f / mag) : Vec2();
	}
	
	//Adding a Vector
	void operator += (const Vec2& v)
	{
		x += v.x; 
		y += v.y;
	}
	
	//Subtracting a vector
	void operator -= (const Vec2& v)
	{
		x -= v.x;	
		y -= v.y;
	}

	//Multiplicando Vector Por Escalar
	void operator *= (const real& s)
	{
		x *= s;
		y *= s;
	}

	//Sum of Vectors
	Vec2 operator + (const Vec2& v)
	{
		return Vec2(x + v.x, y + v.y);	
	}
		
	//Subtraction of Vectors
	Vec2 operator - (const Vec2& v)
	{
		return Vec2(x - v.x, y - v.y);
	}
	
	//Negation of vector
	Vec2 operator - (void)
	{
		return Vec2(-x, -y);
	}
		
	//Set the components to Absolute Value.
	void SetAbs(void)
	{
		x = absf(x);
		y = absf(y);
	}
	
	//Get the components in Absolute Value.
	Vec2 GetAbs(void)
	{
		return Vec2(absf(x), absf(y));
	}
	
	/****************************************************************************
	One of the most important operations of vectors is the dot product.
	The dot product tells us how vector A in vector B is pointing.
	Avoid making expensive projections and calculating square roots.
	This operation will be widely used both in collision detection and in physics.

	if(x1 * x2 + y1 * y2 == 0) then the vectors are perpendicular. Rigth angle.
	if(x1 * x2 + y1 * y2 >  0) then vectors point in the same direction. acute. 
	if(x1 * x2 + y1 * y2 <  0) then vectors point in the opposite direction. obtuse. 
	*****************************************************************************/
	real operator * (const Vec2 &v)
	{
		return (x * v.x + y * v.y);
	}
	
	//Vector Product By A Scalar
	Vec2 operator * (const real& s)
	{
		return Vec2 (x * s, y * s);
	}
	
	//Set the Vector Components to Zero
	void SetZero(void) {x = y = 0.0f;}
};


/****************************************************************************
One of the most important operations of vectors is the dot product.
The dot product tells us how vector A in vector B is pointing.
Avoid making expensive projections and calculating square roots.
This operation will be widely used both in collision detection and in physics.

if(x1 * x2 + y1 * y2 == 0) then the vectors are perpendicular. Rigth angle.
if(x1 * x2 + y1 * y2  > 0) then vectors point in the same direction. acute. 
if(x1 * x2 + y1 * y2  < 0) then vectors point in the opposite direction. obtuse. 
*****************************************************************************/
inline real Dot(const Vec2 &v1, const Vec2& v2)
{
	return v1.x * v2.x + v1.y * v2.y;
}


/****************************************************************************
Una de las operaciones mas importantes de los vectores es el producto cruzado.
se puede observar que es la misma ecuacion que la determinante de una matriz 2x2 or x1 * y2 - y1 * x2. 
ecuacion lineal con dos incognitas. lo cual indica que:

if (x1 * y2 - y1 * x2 != 0) then The vectors are not parallel, there is only one solution.
if (x1 * y2 - y1 * x2 == 0) then The vectors are parallel or are the same vector. It has no solution or has an infinite number of solutions.

i = X; 
j = Y; 
k = Z;

 k = i X j --> Not counterclockwise
-k = j X i --> counterclockwise

Cross(v1, v2) = Dot(v1 Perp , v2);

The Function was defined to go counterclockwise.
****************************************************************************/
inline real Cross(const Vec2 v1, const Vec2 v2)
{
	return v1.x * v2.y - v1.y * v2.x;
}

//Producto Cruzado entre Vector y escalar - Direccion Antihorario
inline Vec2 Cross(const Vec2 v, const real& s)
{
	return Vec2(s * v.y, -s * v.x);
}

inline real Clamp(const real min, const real max, const real& v)
{
  if (v < min) return min;
  if (v > max) return max;
	return v;
}


/*********************************************************************************
The rotation matrices are square matrices, with real entries. these matrices describe rotations about the origin.
The matrices also have their respective operations such as: addition, subtraction and multiplication.
but this Mat2 class has only operations of the rotation matrix.

The Matrix 2x2 rotation matrices is:

	| cos(θ)   sin(θ)|
	|-sin(θ)	 cos(θ)| 

We only need the distance in Cartesian coordinates and the orientation in polar.

	| cos(θ)   sin(θ)| |x|
	|-sin(θ)   cos(θ)| |y|

We multiply the matrix by the vector and obtain the new components of the vector with respect to rotation.

	 x =  cos(0) * x + sin(θ) * y
	 y = -sin(0) * x + cos(0) * y

The direction of rotation of this matrix is counterclockwise

**********************************************************************************/

class Mat2
{
private:
	
	real m00, m01;
	real m10, m11;

public:

	//Constructor by DeFault of the Class
	Mat2() : m00(0.0f), m01(0.0f), m10(0.0f), m11(0.0f) {}

	//Constructor Initialized of the Class
	Mat2(const real& rad)
	{
		m00 =  cosf(rad);  m01 = sinf(rad);

		m10 = -m01;        m11 = m00;       
	}
		
	//Return the Matriz of Rotation Radian - Magnitude
	Vec2 Rotate(const real& rad, const Vec2& v)
	{
		m00 = cosf(rad);  m01 = sinf(rad);

		m10 = -m01;       m11 = m00;

		return Vec2(v.x * m00 + v.y * m01, v.x * m10 + v.y * m11);
	}

	//Return The Matriz of Rotation
	Vec2 Rotate(const Vec2& v)
	{
		return Vec2(v.x * m00 + v.y * m01, v.x * m10 + v.y * m11);
	}

	//Return The Matriz of Rotation
	Vec2 operator * (const Vec2& v)
	{
		return Vec2(v.x * m00 + v.y * m01, v.x * m10 + v.y * m11);
	}

	//Return The Column 0 of The Matriz
	Vec2 Column0(void) {return Vec2(m00, m10);}

	//Return The Column 1 Of The Matriz
	Vec2 Column1(void) {return Vec2(m01, m11);}

	//set the matrix transpose
	void Transpose(void) {real temp = m01; m01 = m10; m10 = temp;}

	//Set The Vector with Matrix Transpose
	void SetTranspose(Vec2& v) 
	{
		real  x = v.x,  y = v.y;
		v.x = x * m00 + y * m10;
		v.y = x * m01 + y * m11;
	}	

	//Return the Matrix Transpose
	Vec2 Transpose(const Vec2& v)
	{
		return Vec2(v.x * m00 + v.y * m10, v.x * m01 + v.y * m11);
	}
};

#endif














