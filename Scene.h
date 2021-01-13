#ifndef SCENE_H
#define SCENE_H

#include <map>
#include <vector>
#include "Manifold.h"

enum {NONE = 0, BAUMGARTE, NGS};

struct Scene
{
	static int CorrectionType;

	Vec2 gravity;

	int iterPos, iterVel;

	std::vector<RigidBody*> bodies;

	std::map<ManifoldKey, Manifold> manifolds;

	Scene(const Vec2& _gravity, const int& _iterVel, const int& _iterPos) : gravity(_gravity), iterVel(_iterVel), iterPos(_iterPos) {;}

	void Clear(void);

	void Add(RigidBody*);

	void Step(const real&);

	void BroadPhase(void);
};

#endif
