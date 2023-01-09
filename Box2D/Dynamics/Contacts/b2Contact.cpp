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
#include "Box2D/Dynamics/Contacts/b2Contact.h"
#include "Box2D/Common/b2BlockAllocator.h"
#include "Box2D/Dynamics/b2Body.h"
#include "Box2D/Collision/b2Collision.h"
#include "Box2D/Dynamics/b2Fixture.h"
#include "Box2D/Collision/Shapes/b2Shape.h"
#include "Box2D/Collision/b2TimeOfImpact.h"
#include "Box2D/Dynamics/b2World.h"

#define b2CreateCollisionFuncWrapper(FUNC_NAME, TYPE_A, TYPE_B)                        \
void FUNC_NAME##Wrapper (b2Manifold* manifold,                                         \
	const b2Shape* shapeA, const b2Transform& xfA,                 \
	const b2Shape* shapeB, const b2Transform& xfB) {               \
	FUNC_NAME(manifold, (const TYPE_A*) (shapeA), xfA, (const TYPE_B*) (shapeB), xfB);   \
}

// Create b2EvaluateFunction wrapper functions for the collision functions
b2CreateCollisionFuncWrapper(b2CollideCircles, b2CircleShape, b2CircleShape)
b2CreateCollisionFuncWrapper(b2CollidePolygonAndCircle, b2PolygonShape, b2CircleShape)
b2CreateCollisionFuncWrapper(b2CollidePolygons, b2PolygonShape, b2PolygonShape)
b2CreateCollisionFuncWrapper(b2CollideEdgeAndCircle, b2EdgeShape, b2CircleShape)
b2CreateCollisionFuncWrapper(b2CollideEdgeAndPolygon, b2EdgeShape, b2PolygonShape)

b2EvaluateFunction* b2Contact::functions[b2Shape::e_typeCount][b2Shape::e_typeCount];

bool b2Contact::InitializeRegisters() {
	functions[b2Shape::e_circle][b2Shape::e_circle] = &b2CollideCirclesWrapper;
	functions[b2Shape::e_polygon][b2Shape::e_circle] = &b2CollidePolygonAndCircleWrapper;
	functions[b2Shape::e_polygon][b2Shape::e_polygon] = &b2CollidePolygonsWrapper;
	functions[b2Shape::e_edge][b2Shape::e_circle] = &b2CollideEdgeAndCircleWrapper;
	functions[b2Shape::e_edge][b2Shape::e_polygon] = &b2CollideEdgeAndPolygonWrapper;

	return true;
}

b2Contact* b2Contact::Create(b2Fixture* fixtureA, b2Fixture* fixtureB, b2BlockAllocator* allocator) {
	// lambda to initialize once
	static bool initialize = []() { return InitializeRegisters(); } ();

	b2Shape::Type type1 = fixtureA->GetType();
	b2Shape::Type type2 = fixtureB->GetType();

	b2Assert(0 <= type1 && type1 < b2Shape::e_typeCount);
	b2Assert(0 <= type2 && type2 < b2Shape::e_typeCount);

	if (functions[type1][type2] != nullptr) {
		void* mem = allocator->Allocate(sizeof(b2Contact));
		return new (mem) b2Contact(fixtureA, fixtureB, functions[type1][type2]);
	} else if (functions[type2][type1] != nullptr) {
		void* mem = allocator->Allocate(sizeof(b2Contact));
		return new (mem) b2Contact(fixtureB, fixtureA, functions[type2][type1]);
	} else {
		return nullptr;
	}
}

void b2Contact::Destroy(b2Contact* contact, b2BlockAllocator* allocator) {
#ifdef ENABLE_SLEEPING
	b2Fixture* fixtureA = contact->m_fixtureA;
	b2Fixture* fixtureB = contact->m_fixtureB;

	if (contact->m_manifold.pointCount > 0 &&
		fixtureA->IsSensor() == false &&
		fixtureB->IsSensor() == false)
	{
		fixtureA->GetBody()->SetAwake(true);
		fixtureB->GetBody()->SetAwake(true);
	}
#endif // ENABLE_SLEEPING

	allocator->Free(contact, sizeof(b2Contact));
}

b2Contact::b2Contact(b2Fixture* fA, b2Fixture* fB, b2EvaluateFunction* evaluateFunction) {
	m_flags = e_enabledFlag | e_persistFlag;

	m_fixtureA = fA;
	m_fixtureB = fB;
	m_evaluateFunction = evaluateFunction;

	m_manifold.pointCount = 0;

	m_prev = nullptr;
	m_next = nullptr;

	m_toiIndex = -1;

#ifdef ENABLE_FRICTION
	m_friction = b2MixFriction(m_fixtureA->m_friction, m_fixtureB->m_friction);
#endif // ENABLE_FRICTION

#ifdef ENABLE_RESTITUTION
	m_restitution = b2MixRestitution(m_fixtureA->m_restitution, m_fixtureB->m_restitution);
	m_restitutionThreshold = b2MixRestitutionThreshold(m_fixtureA->m_restitutionThreshold, m_fixtureB->m_restitutionThreshold);
#endif // ENABLE_RESTITUTION

#ifdef ENABLE_TANGENT_SPEED
	m_tangentSpeed = 0.0f;
#endif // ENABLE_TANGENT_SPEED
}

// Update the contact manifold and touching status.
// Note: do not assume the fixture AABBs are overlapping or are valid.
void b2Contact::Update(b2ContactListener* listener) {
	b2Manifold oldManifold = m_manifold;

	// Re-enable this contact.
	m_flags |= e_enabledFlag;

	bool touching = false;
	bool wasTouching = IsTouching();

	bool sensorA = m_fixtureA->IsSensor();
	bool sensorB = m_fixtureB->IsSensor();
	bool sensor = sensorA || sensorB;

	b2Body* bodyA = m_fixtureA->GetBody();
	b2Body* bodyB = m_fixtureB->GetBody();
	const b2Transform& xfA = bodyA->GetTransform();
	const b2Transform& xfB = bodyB->GetTransform();

	// Is this contact a sensor?
	if (sensor) {
		const b2Shape* shapeA = m_fixtureA->GetShape();
		const b2Shape* shapeB = m_fixtureB->GetShape();
		touching = b2TestOverlap(shapeA, shapeB, xfA, xfB);

		// Sensors don't generate manifolds.
		m_manifold.pointCount = 0;
	} else {
		Evaluate(&m_manifold, xfA, xfB);
		touching = m_manifold.pointCount > 0;

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for (int32 i = 0; i < m_manifold.pointCount; ++i) {
			b2ManifoldPoint* mp2 = m_manifold.points + i;
			mp2->normalImpulse = 0.0f;

#ifdef ENABLE_FRICTION
			mp2->tangentImpulse = 0.0f;
#endif // ENABLE_FRICTION

			b2ContactID id2 = mp2->id;

			for (int32 j = 0; j < oldManifold.pointCount; ++j) {
				b2ManifoldPoint* mp1 = oldManifold.points + j;

				if (mp1->id.key == id2.key) {
					mp2->normalImpulse = mp1->normalImpulse;

#ifdef ENABLE_FRICTION
					mp2->tangentImpulse = mp1->tangentImpulse;
#endif // ENABLE_FRICTION

					break;
				}
			}
		}

#ifdef ENABLE_SLEEPING
		if (touching != wasTouching) {
			bodyA->SetAwake(true);
			bodyB->SetAwake(true);
		}
#endif // ENABLE_SLEEPING
	}

	if (touching) {
		m_flags |= e_touchingFlag;
	} else {
		m_flags &= ~e_touchingFlag;
	}

	if (listener) {
		if (wasTouching == false && touching == true) {
			listener->BeginContact(this);
		}

		if (wasTouching == true && touching == false) {
			listener->EndContact(this);
		}

		if (sensor == false && touching == true) {
			listener->PreSolve(this, &oldManifold);
		}
	}
}

float b2Contact::CalculateTOI() {
	b2Fixture* fA = GetFixtureA();
	b2Fixture* fB = GetFixtureB();

	// Is there a sensor?
	if (fA->IsSensor() || fB->IsSensor() || IsEnabled() == false) {
		return 1.0f;
	}

	b2Body* bA = fA->GetBody();
	b2Body* bB = fB->GetBody();

	b2BodyType typeA = bA->m_type;
	b2BodyType typeB = bB->m_type;
	b2Assert(typeA == b2_dynamicBody || typeB == b2_dynamicBody);

#ifdef ENABLE_SLEEPING
	bool activeA = bA->IsAwake() && typeA != b2_staticBody;
	bool activeB = bB->IsAwake() && typeB != b2_staticBody;
	bool active = activeA && activeB;
#else
	bool active = (typeA != b2_staticBody) && (typeB != b2_staticBody);
#endif // ENABLE_SLEEPING

	// Is at least one body active (awake and dynamic or kinematic)?
	if (active == false) {
		return 1.0f;
	}

	bool collideA = bA->IsBullet() || typeA != b2_dynamicBody;
	bool collideB = bB->IsBullet() || typeB != b2_dynamicBody;

	// Are these two non-bullet dynamic bodies?
	if (collideA == false && collideB == false) {
		return 1.0f;
	}

	// Compute the TOI for this contact.
	// Put the sweeps onto the same time interval.
	float alpha0 = bA->m_sweep.alpha0;

	if (bA->m_sweep.alpha0 < bB->m_sweep.alpha0) {
		alpha0 = bB->m_sweep.alpha0;
		bA->m_sweep.Advance(alpha0);
	} else if (bB->m_sweep.alpha0 < bA->m_sweep.alpha0) {
		alpha0 = bA->m_sweep.alpha0;
		bB->m_sweep.Advance(alpha0);
	}

	b2Assert(alpha0 < 1.0f);

	// Compute the time of impact in interval [0, minTOI]
	b2TOIInput input;
	input.proxyA.Set(fA->GetShape());
	input.proxyB.Set(fB->GetShape());
	input.sweepA = bA->m_sweep;
	input.sweepB = bB->m_sweep;
	input.tMax = 1.0f;

	b2TOIOutput output;
	b2TimeOfImpact(&output, &input);

	// Beta is the fraction of the remaining portion of the .
	float alpha;
	float beta = output.t;
	if (output.state == b2TOIOutput::e_touching) {
		alpha = b2Min(alpha0 + (1.0f - alpha0) * beta, 1.0f);
	} else {
		alpha = 1.0f;
	}

	return alpha;
}
