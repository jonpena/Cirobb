#ifndef SCENE_H
#define SCENE_H

#include <map>
#include <vector>
#include "Manifold.h"


struct Scene
{
	Vec2 gravity;

	int iterations;

	std::vector<Shape*> bodies;

	std::map<ManifoldKey, Manifold> manifolds;

	Scene(Vec2 _gravity, int _iterations) : gravity(_gravity), iterations(_iterations) {;}

	void Clear(void);

	void Add(Shape*);

	void Step(const real&);

	void BroadPhase(void);
};

#endif
