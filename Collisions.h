#ifndef COLLISIONS_H
#define COLLISIONS_H

#include "Manifold.h"

typedef void(*CollisionCallBack)(Manifold&, Shape*, Shape*);

void CircleToCircle(Manifold&, Shape*, Shape*);

void CircleToOBB(Manifold&, Shape*, Shape*);

void OBBToCircle(Manifold&, Shape*, Shape*);

void OBBToOBB(Manifold&, Shape*, Shape*);

static CollisionCallBack Dispatcher[2][2] = {CircleToCircle, CircleToOBB, OBBToCircle, OBBToOBB};

#endif