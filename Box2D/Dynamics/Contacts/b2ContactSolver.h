/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_CONTACT_SOLVER_H
#define B2_CONTACT_SOLVER_H

#include "Box2D/Collision/b2Collision.h"
#include "Box2D/Common/b2Math.h"
#include "Box2D/Dynamics/b2TimeStep.h"

class b2Contact;
class b2Body;
class b2StackAllocator;
struct b2ContactPositionConstraint;

struct b2VelocityConstraintPoint {
	b2Vec2 rA;
	b2Vec2 rB;
	float normalImpulse;
	float normalMass;

#ifdef ENABLE_FRICTION
	float tangentImpulse;
	float tangentMass;
#endif // ENABLE_FRICTION

#ifdef ENABLE_RESTITUTION
	float velocityBias;
#endif // ENABLE_RESTITUTION
};

struct b2ContactVelocityConstraint {
	b2VelocityConstraintPoint points[b2_maxManifoldPoints];
	b2Vec2 normal;
	b2Mat22 normalMass;
	b2Mat22 K;
	int32 indexA;
	int32 indexB;
	float invMassA, invMassB;
	float invIA, invIB;

#ifdef ENABLE_FRICTION
	float friction;
#endif // ENABLE_FRICTION

#ifdef ENABLE_RESTITUTION
	float restitution;
	float threshold;
#endif // ENABLE_RESTITUTION

#ifdef ENABLE_TANGENT_SPEED
	float tangentSpeed;
#endif // ENABLE_TANGENT_SPEED

	int32 pointCount;
	b2Manifold* manifold;
};

struct b2ContactSolverDef {
	b2TimeStep step;
	b2Contact** contacts;
	int32 count;
	b2Position* positions;
	b2Velocity* velocities;
	b2StackAllocator* allocator;
};

class b2ContactSolver
{
public:
	b2ContactSolver();
	~b2ContactSolver();

	void Initialize(b2ContactSolverDef* def);
	void InitializeVelocityConstraints();

	void WarmStart();
	void SolveVelocityConstraints();
	void StoreImpulses();

	bool SolvePositionConstraints();
	bool SolveTOIPositionConstraints(int32 toiIndexA, int32 toiIndexB);

	b2Position* m_positions;
	b2Velocity* m_velocities;
	b2StackAllocator* m_allocator;
	b2ContactPositionConstraint* m_positionConstraints;
	b2ContactVelocityConstraint* m_velocityConstraints;
	int m_count;
};

#endif

