/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include "Box2D/Common/b2BlockAllocator.h"
#include "Box2D/Dynamics/b2Body.h"
#include "Box2D/Dynamics/Joints/b2DistanceJoint.h"
#include "Box2D/Common/b2Draw.h"
#include "Box2D/Dynamics/Joints/b2FrictionJoint.h"
#include "Box2D/Dynamics/Joints/b2GearJoint.h"
#include "Box2D/Dynamics/Joints/b2MotorJoint.h"
#include "Box2D/Dynamics/Joints/b2MouseJoint.h"
#include "Box2D/Dynamics/Joints/b2PrismaticJoint.h"
#include "Box2D/Dynamics/Joints/b2PulleyJoint.h"
#include "Box2D/Dynamics/Joints/b2RevoluteJoint.h"
#include "Box2D/Dynamics/Joints/b2WeldJoint.h"
#include "Box2D/Dynamics/Joints/b2WheelJoint.h"
#include "Box2D/Dynamics/Joints/b2FakeJoint.h"
#include "Box2D/Dynamics/Joints/b2RopeJoint.h"
#include "Box2D/Dynamics/b2World.h"

#include <new>

void b2LinearStiffness(float& stiffness, float& damping,
	float frequencyHertz, float dampingRatio,
	const b2Body* bodyA, const b2Body* bodyB)
{
	float massA = bodyA->GetMass();
	float massB = bodyB->GetMass();
	float mass;
	if (massA > 0.0f && massB > 0.0f)
	{
		mass = massA * massB / (massA + massB);
	}
	else if (massA > 0.0f)
	{
		mass = massA;
	}
	else
	{
		mass = massB;
	}

	float omega = 2.0f * b2_pi * frequencyHertz;
	stiffness = mass * omega * omega;
	damping = 2.0f * mass * dampingRatio * omega;
}

void b2AngularStiffness(float& stiffness, float& damping,
	float frequencyHertz, float dampingRatio,
	const b2Body* bodyA, const b2Body* bodyB)
{
	float IA = bodyA->GetInertia();
	float IB = bodyB->GetInertia();
	float I;
	if (IA > 0.0f && IB > 0.0f)
	{
		I = IA * IB / (IA + IB);
	}
	else if (IA > 0.0f)
	{
		I = IA;
	}
	else
	{
		I = IB;
	}

	float omega = 2.0f * b2_pi * frequencyHertz;
	stiffness = I * omega * omega;
	damping = 2.0f * I * dampingRatio * omega;
}

b2Joint* b2Joint::Create(const b2JointDef* def, b2BlockAllocator* allocator)
{
	b2Joint* joint = nullptr;

	switch (def->type)
	{
	case e_distanceJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2DistanceJoint));
			joint = new (mem) b2DistanceJoint(static_cast<const b2DistanceJointDef*>(def));
		}
		break;

	case e_mouseJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2MouseJoint));
			joint = new (mem) b2MouseJoint(static_cast<const b2MouseJointDef*>(def));
		}
		break;

	case e_prismaticJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2PrismaticJoint));
			joint = new (mem) b2PrismaticJoint(static_cast<const b2PrismaticJointDef*>(def));
		}
		break;

	case e_revoluteJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2RevoluteJoint));
			joint = new (mem) b2RevoluteJoint(static_cast<const b2RevoluteJointDef*>(def));
		}
		break;

	case e_pulleyJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2PulleyJoint));
			joint = new (mem) b2PulleyJoint(static_cast<const b2PulleyJointDef*>(def));
		}
		break;

	case e_gearJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2GearJoint));
			joint = new (mem) b2GearJoint(static_cast<const b2GearJointDef*>(def));
		}
		break;

	case e_wheelJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2WheelJoint));
			joint = new (mem) b2WheelJoint(static_cast<const b2WheelJointDef*>(def));
		}
		break;

	case e_weldJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2WeldJoint));
			joint = new (mem) b2WeldJoint(static_cast<const b2WeldJointDef*>(def));
		}
		break;

	case e_fakeJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2FakeJoint));
			joint = new (mem) b2FakeJoint(static_cast<const b2FakeJointDef*>(def));
		}
		break;

	case e_frictionJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2FrictionJoint));
			joint = new (mem) b2FrictionJoint(static_cast<const b2FrictionJointDef*>(def));
		}
		break;

	case e_ropeJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2RopeJoint));
			joint = new (mem) b2RopeJoint(static_cast<const b2RopeJointDef*>(def));
		}
		break;

	case e_motorJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2MotorJoint));
			joint = new (mem) b2MotorJoint(static_cast<const b2MotorJointDef*>(def));
		}
		break;

	default:
		b2Assert(false);
		break;
	}

	return joint;
}

void b2Joint::Destroy(b2Joint* joint, b2BlockAllocator* allocator)
{
	joint->~b2Joint();
	switch (joint->m_type)
	{
	case e_distanceJoint:
		allocator->Free(joint, sizeof(b2DistanceJoint));
		break;

	case e_mouseJoint:
		allocator->Free(joint, sizeof(b2MouseJoint));
		break;

	case e_prismaticJoint:
		allocator->Free(joint, sizeof(b2PrismaticJoint));
		break;

	case e_revoluteJoint:
		allocator->Free(joint, sizeof(b2RevoluteJoint));
		break;

	case e_pulleyJoint:
		allocator->Free(joint, sizeof(b2PulleyJoint));
		break;

	case e_gearJoint:
		allocator->Free(joint, sizeof(b2GearJoint));
		break;

	case e_wheelJoint:
		allocator->Free(joint, sizeof(b2WheelJoint));
		break;
    
	case e_weldJoint:
		allocator->Free(joint, sizeof(b2WeldJoint));
		break;

	case e_frictionJoint:
		allocator->Free(joint, sizeof(b2FrictionJoint));
		break;

	case e_ropeJoint:
		allocator->Free(joint, sizeof(b2RopeJoint));
		break;

	case e_motorJoint:
		allocator->Free(joint, sizeof(b2MotorJoint));
		break;

	case e_fakeJoint:
		allocator->Free(joint, sizeof(b2FakeJoint));
		break;

	default:
		b2Assert(false);
		break;
	}
}

b2Joint::b2Joint(const b2JointDef* def)
{
	b2Assert(def->bodyA != def->bodyB);

	m_type = def->type;
	m_prev = nullptr;
	m_next = nullptr;
	m_bodyA = def->bodyA;
	m_bodyB = def->bodyB;
	m_index = 0;
	m_collideConnected = def->collideConnected;
	m_islandFlag = false;
	m_userData = def->userData;

	m_edgeA.joint = nullptr;
	m_edgeA.other = nullptr;
	m_edgeA.prev = nullptr;
	m_edgeA.next = nullptr;

	m_edgeB.joint = nullptr;
	m_edgeB.other = nullptr;
	m_edgeB.prev = nullptr;
	m_edgeB.next = nullptr;
}

bool b2Joint::IsEnabled() const
{
	return m_bodyA->IsEnabled() && m_bodyB->IsEnabled();
}

void b2Joint::Draw(b2Draw* draw) const
{
	const b2Transform& xf1 = m_bodyA->GetTransform();
	const b2Transform& xf2 = m_bodyB->GetTransform();
	b2Vec2 x1 = xf1.p;
	b2Vec2 x2 = xf2.p;
	b2Vec2 p1 = GetAnchorA();
	b2Vec2 p2 = GetAnchorB();

	b2Color color(0.5f, 0.8f, 0.8f);

	switch (m_type)
	{
		case e_distanceJoint:
		draw->DrawSegment(p1, p2, color);
		break;

		case e_pulleyJoint:
		{
			b2PulleyJoint* pulley = (b2PulleyJoint*)this;
			b2Vec2 s1 = pulley->GetGroundAnchorA();
			b2Vec2 s2 = pulley->GetGroundAnchorB();
			draw->DrawSegment(s1, p1, color);
			draw->DrawSegment(s2, p2, color);
			draw->DrawSegment(s1, s2, color);
		}
		break;

		case e_mouseJoint:
		{
			b2Color c;
			c.Set(0.0f, 1.0f, 0.0f);
			draw->DrawPoint(p1, 4.0f, c);
			draw->DrawPoint(p2, 4.0f, c);

			c.Set(0.8f, 0.8f, 0.8f);
			draw->DrawSegment(p1, p2, c);

		}
		break;

		default:
		draw->DrawSegment(x1, p1, color);
		draw->DrawSegment(p1, p2, color);
		draw->DrawSegment(x2, p2, color);
	}
}
