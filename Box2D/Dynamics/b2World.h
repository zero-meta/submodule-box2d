/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
* Copyright (c) 2013 Google, Inc.
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

#ifndef B2_WORLD_H
#define B2_WORLD_H

#include "Box2D/Common/b2BlockAllocator.h"
#include "Box2D/Dynamics/b2ContactManager.h"
#include "Box2D/Common/b2Math.h"
#include "Box2D/Common/b2StackAllocator.h"
#include "Box2D/Dynamics/b2TimeStep.h"
#include "Box2D/Dynamics/b2WorldCallbacks.h"
#include "Box2D/Particle/b2ParticleSystem.h"

struct b2AABB;
struct b2BodyDef;
struct b2Color;
struct b2JointDef;
class b2Body;
class b2Draw;
class b2Fixture;
class b2Joint;

class b2ParticleGroup;

/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.
class B2_API b2World
{
public:
	/// Construct a world object.
	/// @param gravity the world gravity vector.
	b2World(const b2Vec2& gravity);

	/// Destruct the world. All physics entities are destroyed and all heap memory is released.
	~b2World();

	/// Register a destruction listener. The listener is owned by you and must
	/// remain in scope.
	void SetDestructionListener(b2DestructionListener* listener);

	/// Register a contact filter to provide specific control over collision.
	/// Otherwise the default filter is used (b2_defaultFilter). The listener is
	/// owned by you and must remain in scope.
	void SetContactFilter(b2ContactFilter* filter);

	/// Register a contact event listener. The listener is owned by you and must
	/// remain in scope.
	void SetContactListener(b2ContactListener* listener);

	/// Register a routine for debug drawing. The debug draw functions are called
	/// inside with b2World::DrawDebugData method. The debug draw object is owned
	/// by you and must remain in scope.
	void SetDebugDraw(b2Draw* debugDraw);

	/// Create a rigid body given a definition. No reference to the definition
	/// is retained.
	/// @warning This function is locked during callbacks.
	b2Body* CreateBody(const b2BodyDef* def);

	/// Destroy a rigid body given a definition. No reference to the definition
	/// is retained. This function is locked during callbacks.
	/// @warning This automatically deletes all associated shapes and joints.
	/// @warning This function is locked during callbacks.
	void DestroyBody(b2Body* body);

	/// Create a joint to constrain bodies together. No reference to the definition
	/// is retained. This may cause the connected bodies to cease colliding.
	/// @warning This function is locked during callbacks.
	b2Joint* CreateJoint(const b2JointDef* def);

	/// Destroy a joint. This may cause the connected bodies to begin colliding.
	/// @warning This function is locked during callbacks.
	void DestroyJoint(b2Joint* joint);

#ifdef ENABLE_LIQUID
	/// Create a particle system given a definition. No reference to the
	/// definition is retained.
	/// @warning This function is locked during callbacks.
	b2ParticleSystem* CreateParticleSystem(const b2ParticleSystemDef* def);

	/// Destroy a particle system.
	/// @warning This function is locked during callbacks.
	void DestroyParticleSystem(b2ParticleSystem* p);

	/// Get the world particle-system list. With the returned body, use
	/// b2ParticleSystem::GetNext to get the next particle-system in the world
	/// list. A NULL particle-system indicates the end of the list.
	/// @return the head of the world particle-system list.
	b2ParticleSystem* GetParticleSystemList();
	const b2ParticleSystem* GetParticleSystemList() const;
#endif // ENABLE_LIQUID

	/// Take a time step. This performs collision detection, integration,
	/// and constraint solution.
	/// For the numerical stability of particles, minimize the following
	/// dimensionless gravity acceleration:
	///     gravity / particleRadius * (timeStep / particleIterations)^2
	/// b2CalculateParticleIterations() or
	/// CalculateReasonableParticleIterations() help to determine the optimal
	/// particleIterations.
	/// @param timeStep the amount of time to simulate, this should not vary.
	/// @param velocityIterations for the velocity constraint solver.
	/// @param positionIterations for the position constraint solver.
	/// @param particleIterations for the particle simulation.
	void Step(	float timeStep,
				int32 velocityIterations,
				int32 positionIterations,
				int32 particleIterations);

	/// Take a time step. This performs collision detection, integration,
	/// and constraint solution.
	/// @param timeStep the amount of time to simulate, this should not vary.
	/// @param velocityIterations for the velocity constraint solver.
	/// @param positionIterations for the position constraint solver.
	void Step(	float timeStep,
				int32 velocityIterations,
				int32 positionIterations)
	{
		Step(timeStep, velocityIterations, positionIterations, 1);
	}

	/// Recommend a value to be used in `Step` for `particleIterations`.
	/// This calculation is necessarily a simplification and should only be
	/// used as a starting point. Please see "Particle Iterations" in the
	/// Programmer's Guide for details.
	/// @param timeStep is the value to be passed into `Step`.
	// int CalculateReasonableParticleIterations(float32 timeStep) const;

	/// Manually clear the force buffer on all bodies. By default, forces are cleared automatically
	/// after each call to Step. The default behavior is modified by calling SetAutoClearForces.
	/// The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
	/// a fixed sized time step under a variable frame-rate.
	/// When you perform sub-stepping you will disable auto clearing of forces and instead call
	/// ClearForces after all sub-steps are complete in one pass of your game loop.
	/// @see SetAutoClearForces
	void ClearForces();

	/// Call this to draw shapes and other debug draw data. This is intentionally non-const.
	void DebugDraw();

	/// Query the world for all fixtures that potentially overlap the
	/// provided AABB.
	/// @param callback a user implemented callback class.
	/// @param aabb the query box.
	void QueryAABB(b2QueryCallback* callback, const b2AABB& aabb);

	/// Ray-cast the world for all fixtures in the path of the ray. Your callback
	/// controls whether you get the closest point, any point, or n-points.
	/// The ray-cast ignores shapes that contain the starting point.
	/// @param callback a user implemented callback class.
	/// @param point1 the ray starting point
	/// @param point2 the ray ending point
	void RayCast(b2RayCastCallback* callback, const b2Vec2& point1, const b2Vec2& point2);

	/// Get the world body list. With the returned body, use b2Body::GetNext to get
	/// the next body in the world list. A nullptr body indicates the end of the list.
	/// @return the head of the world body list.
	b2Body* GetBodyList();
	const b2Body* GetBodyList() const;

	/// Get the world joint list. With the returned joint, use b2Joint::GetNext to get
	/// the next joint in the world list. A nullptr joint indicates the end of the list.
	/// @return the head of the world joint list.
	b2Joint* GetJointList();
	const b2Joint* GetJointList() const;

	/// Get the world contact list. With the returned contact, use b2Contact::GetNext to get
	/// the next contact in the world list. A special marker contact, returned by GetContactListEnd, indicates the end of the list.
	/// @return the head of the world contact list.
	/// @warning contacts are created and destroyed in the middle of a time step.
	/// Use b2ContactListener to avoid missing contacts.
	b2Contact* GetContactListStart();
	const b2Contact* GetContactListStart() const;

	/// @return a special marker contact that marks the end of iteration for the contact list.
	b2Contact* GetContactListEnd();
	const b2Contact* GetContactListEnd() const;

	/// Enable/disable sleep.
	void SetAllowSleeping(bool flag);
	bool GetAllowSleeping() const { return m_allowSleep; }

	/// Enable/disable warm starting. For testing.
	void SetWarmStarting(bool flag) { m_warmStarting = flag; }
	bool GetWarmStarting() const { return m_warmStarting; }

	/// Enable/disable continuous physics. For testing.
	void SetContinuousPhysics(bool flag) { m_continuousPhysics = flag; }
	bool GetContinuousPhysics() const { return m_continuousPhysics; }

	/// Enable/disable single stepped continuous physics. For testing.
	void SetSubStepping(bool flag) { m_subStepping = flag; }
	bool GetSubStepping() const { return m_subStepping; }

	/// Get the number of broad-phase proxies.
	int32 GetProxyCount() const;

	/// Get the number of bodies.
	int32 GetBodyCount() const;

	/// Get the number of joints.
	int32 GetJointCount() const;

	/// Get the number of contacts (each may have 0 or more contact points).
	int32 GetContactCount() const;

	/// Get the height of the dynamic tree.
	int32 GetTreeHeight() const;

	/// Change the global gravity vector.
	void SetGravity(const b2Vec2& gravity);

	/// Get the global gravity vector.
	b2Vec2 GetGravity() const;

	/// Is the world locked (in the middle of a time step).
	bool IsLocked() const;

	/// Set flag to control automatic clearing of forces after each time step.
	void SetAutoClearForces(bool flag);

	/// Get the flag that controls automatic clearing of forces after each time step.
	bool GetAutoClearForces() const;

	/// Shift the world origin. Useful for large worlds.
	/// The body shift formula is: position -= newOrigin
	/// @param newOrigin the new origin with respect to the old origin
	void ShiftOrigin(const b2Vec2& newOrigin);

	/// Get the contact manager for testing.
	const b2ContactManager& GetContactManager() const;

	/// Get the current profile.
	const b2Profile& GetProfile() const;

	/// Dump the world into the log file.
	/// @warning this should be called outside of a time step.
	void Dump();

#if LIQUIDFUN_EXTERNAL_LANGUAGE_API
public:
	/// Constructor which takes direct floats.
	b2World(float32 gravityX, float32 gravityY);

	/// Set gravity with direct floats.
	void SetGravity(float32 gravityX, float32 gravityY);
#endif // LIQUIDFUN_EXTERNAL_LANGUAGE_API

private:

	friend class b2Body;
	friend class b2Fixture;
	friend class b2ContactManager;
	friend class b2Controller;

	friend class b2ParticleSystem;

	void RemoveDeadContacts();

	void Solve(const b2TimeStep& step);
	void SolveTOI(const b2TimeStep& step);
	float CalculateTOI(b2Contact* c);

	void DrawShape(b2Fixture* shape, const b2Transform& xf, const b2Color& color);

	void DrawParticleSystem(const b2ParticleSystem& system);

	b2BlockAllocator m_blockAllocator;
	b2StackAllocator m_stackAllocator;

	b2ContactManager m_contactManager;

	b2Body* m_bodyListHead;
	b2Body* m_bodyListTail;
	b2Joint* m_jointList;
	b2ParticleSystem* m_particleSystemList;

	int32 m_bodyCount;
	int32 m_jointCount;

	b2Vec2 m_gravity;
	bool m_allowSleep;

	b2DestructionListener* m_destructionListener;
	b2Draw* m_debugDraw;

	// This is used to compute the time step ratio to
	// support a variable time step.
	float m_inv_dt0;

	bool m_newContacts;
	bool m_removedBodies;
	bool m_locked;
	bool m_clearForces;

	// These are for debugging the solver.
	bool m_warmStarting;
	bool m_continuousPhysics;
	bool m_subStepping;

	bool m_stepComplete;

	b2Profile m_profile;
};

inline b2Body* b2World::GetBodyList()
{
	return m_bodyListHead;
}

inline const b2Body* b2World::GetBodyList() const
{
	return m_bodyListHead;
}

inline b2Joint* b2World::GetJointList()
{
	return m_jointList;
}

inline const b2Joint* b2World::GetJointList() const
{
	return m_jointList;
}

inline b2Contact* b2World::GetContactListStart() {
	return m_contactManager.Start();
}

inline const b2Contact* b2World::GetContactListStart() const {
	return m_contactManager.Start();
}

inline b2Contact* b2World::GetContactListEnd() {
	return m_contactManager.End();
}

inline const b2Contact* b2World::GetContactListEnd() const {
	return m_contactManager.End();
}

#ifdef ENABLE_LIQUID
inline b2ParticleSystem* b2World::GetParticleSystemList()
{
	return m_particleSystemList;
}

inline const b2ParticleSystem* b2World::GetParticleSystemList() const
{
	return m_particleSystemList;
}
#endif // ENABLE_LIQUID

inline int32 b2World::GetBodyCount() const
{
	return m_bodyCount;
}

inline int32 b2World::GetJointCount() const
{
	return m_jointCount;
}

inline int32 b2World::GetContactCount() const
{
	return m_contactManager.m_contactCount;
}

inline void b2World::SetGravity(const b2Vec2& gravity)
{
	m_gravity = gravity;
}

inline b2Vec2 b2World::GetGravity() const
{
	return m_gravity;
}

inline bool b2World::IsLocked() const
{
	return m_locked;
}

inline void b2World::SetAutoClearForces(bool flag)
{
	m_clearForces = flag;
}

/// Get the flag that controls automatic clearing of forces after each time step.
inline bool b2World::GetAutoClearForces() const
{
	return m_clearForces;
}

inline const b2ContactManager& b2World::GetContactManager() const
{
	return m_contactManager;
}

inline const b2Profile& b2World::GetProfile() const
{
	return m_profile;
}

#if LIQUIDFUN_EXTERNAL_LANGUAGE_API
inline b2World::b2World(float32 gravityX, float32 gravityY)
{
	Init(b2Vec2(gravityX, gravityY));
}

inline void b2World::SetGravity(float32 gravityX, float32 gravityY)
{
	SetGravity(b2Vec2(gravityX, gravityY));
}
#endif // LIQUIDFUN_EXTERNAL_LANGUAGE_API

#endif
