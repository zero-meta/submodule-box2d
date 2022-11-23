/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/b2FakeJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

void b2FakeJointDef::Initialize(b2Body* bA, b2Body* bB)
{
	bodyA = bA;
	bodyB = bB;
}

b2FakeJoint::b2FakeJoint(const b2FakeJointDef* def)
: b2Joint(def)
{
}

void b2FakeJoint::InitVelocityConstraints(const b2SolverData& data)
{
	B2_NOT_USED(data);
}

void b2FakeJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	B2_NOT_USED(data);
}

bool b2FakeJoint::SolvePositionConstraints(const b2SolverData& data)
{
	B2_NOT_USED(data);
	return true;
}

b2Vec2 b2FakeJoint::GetAnchorA() const
{
	return m_bodyA->GetPosition();
}

b2Vec2 b2FakeJoint::GetAnchorB() const
{
	return m_bodyB->GetPosition();
}

b2Vec2 b2FakeJoint::GetReactionForce(float32 inv_dt) const
{
	return b2Vec2(0.0f, 0.0f);
}

float32 b2FakeJoint::GetReactionTorque(float32 inv_dt) const
{
	return 0.0f;
}

void b2FakeJoint::Dump()
{
	int32 indexA = m_bodyA->m_islandIndex;
	int32 indexB = m_bodyB->m_islandIndex;

	b2Log("  b2FakeJoint jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	b2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
