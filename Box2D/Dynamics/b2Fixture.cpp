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

#include "Box2D/Dynamics/b2Fixture.h"
#include "Box2D/Common/b2BlockAllocator.h"
#include "Box2D/Collision/b2BroadPhase.h"
#include "Box2D/Collision/Shapes/b2CircleShape.h"
#include "Box2D/Collision/b2Collision.h"
#include "Box2D/Dynamics/Contacts/b2Contact.h"
#include "Box2D/Collision/Shapes/b2EdgeShape.h"
#include "Box2D/Collision/Shapes/b2PolygonShape.h"
#include "Box2D/Dynamics/b2World.h"

b2Fixture::b2Fixture()
{
	m_body = nullptr;
	m_next = nullptr;
	m_shape = nullptr;
	m_density = 0.0f;

	static uint32 idGenerator = 0;
	m_id = idGenerator++;
}

void b2Fixture::Create(b2BlockAllocator* allocator, b2Body* body, const b2FixtureDef* def)
{
#ifdef ENABLE_USER_DATA
	m_userData = def->userData;
#endif // ENABLE_USER_DATA

#ifdef ENABLE_FRICTION
	m_friction = def->friction;
#endif // ENABLE_FRICTION

#ifdef ENABLE_RESTITUTION
	m_restitution = def->restitution;
	m_restitutionThreshold = def->restitutionThreshold;
#endif // ENABLE_RESTITUTION

	m_body = body;
	m_next = nullptr;

	m_filter = def->filter;

	m_isSensor = def->isSensor;

	m_shape = def->shape->Clone(allocator);

	m_density = def->density;
}

void b2Fixture::Destroy(b2BlockAllocator* allocator)
{
	// Free the child shape.
	switch (m_shape->m_type)
	{
		case b2Shape::e_circle:
		{
			b2CircleShape* s = (b2CircleShape*)m_shape;
			s->~b2CircleShape();
			allocator->Free(s, sizeof(b2CircleShape));
		}
		break;

		case b2Shape::e_edge:
		{
			b2EdgeShape* s = (b2EdgeShape*)m_shape;
			s->~b2EdgeShape();
			allocator->Free(s, sizeof(b2EdgeShape));
		}
		break;

		case b2Shape::e_polygon:
		{
			b2PolygonShape* s = (b2PolygonShape*)m_shape;
			s->~b2PolygonShape();
			allocator->Free(s, sizeof(b2PolygonShape));
		}
		break;

		default:
		b2Assert(false);
		break;
	}

	m_shape = nullptr;
}
/*
void b2Fixture::CreateProxies(b2BroadPhase* broadPhase, const b2Transform& xf)
{
	b2Assert(m_proxyCount == 0);

	// Create proxies in the broad-phase.
	m_proxyCount = m_shape->GetChildCount();

	for (int32 i = 0; i < m_proxyCount; ++i)
	{
		b2FixtureProxy* proxy = m_proxies + i;
		m_shape->ComputeAABB(&proxy->aabb, xf, i);
		proxy->proxyId = broadPhase->CreateProxy(proxy->aabb, proxy);
		proxy->fixture = this;
		proxy->childIndex = i;
	}
}

void b2Fixture::DestroyProxies(b2BroadPhase* broadPhase)
{
	// Destroy proxies in the broad-phase.
	for (int32 i = 0; i < m_proxyCount; ++i)
	{
		b2FixtureProxy* proxy = m_proxies + i;
		broadPhase->DestroyProxy(proxy->proxyId);
		proxy->proxyId = b2BroadPhase::e_nullProxy;
	}

	m_proxyCount = 0;
}*/

void b2Fixture::UpdateAABB() {
	m_shape->ComputeAABB(&m_aabb, m_body->m_xf);
}

void b2Fixture::SetFilterData(const b2Filter& filter)
{
	m_filter = filter;

	Refilter();
}

void b2Fixture::Refilter()
{
	if (m_body == nullptr)
	{
		return;
	}

	// Flag associated contacts for filtering.
	for (int32 i = 0; i < m_body->GetContactCount(); ++i)
	{
		b2Contact* contact = m_body->GetContact(i);

		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();
		if (fixtureA == this || fixtureB == this)
		{
			contact->FlagForFiltering();
		}
	}

	b2World* world = m_body->GetWorld();

	if (world == nullptr)
	{
		return;
	}

	// Touch each proxy so that new pairs may be created
	// TODO is this needed? (probably not)
	/*b2BroadPhase* broadPhase = &world->m_contactManager.m_broadPhase;
	for (int32 i = 0; i < m_proxyCount; ++i)
	{
		broadPhase->TouchProxy(m_proxies[i].proxyId);
	}*/
}

void b2Fixture::SetSensor(bool sensor)
{
	if (sensor != m_isSensor)
	{
#ifdef ENABLE_SLEEPING
		m_body->SetAwake(true);
#endif // ENABLE_SLEEPING
		m_isSensor = sensor;
	}
}

void b2Fixture::Dump(int32 bodyIndex)
{
	b2Dump("    b2FixtureDef fd;\n");

#ifdef ENABLE_FRICTION
	b2Dump("    fd.friction = %.9g;\n", m_friction);
#endif // ENABLE_FRICTION

#ifdef ENABLE_RESTITUTION
	b2Dump("    fd.restitution = %.9g;\n", m_restitution);
	b2Dump("    fd.restitutionThreshold = %.9g;\n", m_restitutionThreshold);
#endif // ENABLE_RESTITUTION

	b2Dump("    fd.density = %.9g;\n", m_density);
	b2Dump("    fd.isSensor = bool(%d);\n", m_isSensor);
	b2Dump("    fd.filter.categoryBits = uint16(%d);\n", m_filter.categoryBits);
	b2Dump("    fd.filter.maskBits = uint16(%d);\n", m_filter.maskBits);
	b2Dump("    fd.filter.groupIndex = int16(%d);\n", m_filter.groupIndex);

	switch (m_shape->m_type)
	{
		case b2Shape::e_circle:
		{
			b2CircleShape* s = (b2CircleShape*)m_shape;
			b2Dump("    b2CircleShape shape;\n");
			b2Dump("    shape.m_radius = %.9g;\n", s->m_radius);
			b2Dump("    shape.m_p.Set(%.9g, %.9g);\n", s->m_p.x, s->m_p.y);
		}
		break;

		case b2Shape::e_edge:
		{
			b2EdgeShape* s = (b2EdgeShape*)m_shape;
			b2Dump("    b2EdgeShape shape;\n");
			b2Dump("    shape.m_radius = %.9g;\n", s->m_radius);
			b2Dump("    shape.m_vertex0.Set(%.9g, %.9g);\n", s->m_vertex0.x, s->m_vertex0.y);
			b2Dump("    shape.m_vertex1.Set(%.9g, %.9g);\n", s->m_vertex1.x, s->m_vertex1.y);
			b2Dump("    shape.m_vertex2.Set(%.9g, %.9g);\n", s->m_vertex2.x, s->m_vertex2.y);
			b2Dump("    shape.m_vertex3.Set(%.9g, %.9g);\n", s->m_vertex3.x, s->m_vertex3.y);
			b2Dump("    shape.m_oneSided = bool(%d);\n", s->m_oneSided);
		}
		break;

		case b2Shape::e_polygon:
		{
			b2PolygonShape* s = (b2PolygonShape*)m_shape;
			b2Dump("    b2PolygonShape shape;\n");
			b2Dump("    b2Vec2 vs[%d];\n", b2_maxPolygonVertices);
			for (int32 i = 0; i < s->m_count; ++i)
			{
				b2Dump("    vs[%d].Set(%.9g, %.9g);\n", i, s->m_vertices[i].x, s->m_vertices[i].y);
			}
			b2Dump("    shape.Set(vs, %d);\n", s->m_count);
		}
		break;

	/*
	TODO can e_edge handle chains create from the geometry class?
	case b2Shape::e_chain:
		{
			b2ChainShape* s = (b2ChainShape*)m_shape;
			b2Dump("    b2ChainShape shape;\n");
			b2Dump("    b2Vec2 vs[%d];\n", s->m_count);
			for (int32 i = 0; i < s->m_count; ++i)
			{
				b2Dump("    vs[%d].Set(%.9g, %.9g);\n", i, s->m_vertices[i].x, s->m_vertices[i].y);
			}
			b2Dump("    shape.CreateChain(vs, %d);\n", s->m_count);
			b2Dump("    shape.m_prevVertex.Set(%.9g, %.9g);\n", s->m_prevVertex.x, s->m_prevVertex.y);
			b2Dump("    shape.m_nextVertex.Set(%.9g, %.9g);\n", s->m_nextVertex.x, s->m_nextVertex.y);
		}
		break;
	*/

		default:
		return;
	}

	b2Dump("\n");
	b2Dump("    fd.shape = &shape;\n");
	b2Dump("\n");
	b2Dump("    bodies[%d]->CreateFixture(&fd);\n", bodyIndex);
}
