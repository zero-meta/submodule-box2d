/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef B2_CIRCLE_SHAPE_H
#define B2_CIRCLE_SHAPE_H

#include "Box2D/Collision/Shapes/b2Shape.h"

/// A solid circle shape
class B2_API b2CircleShape : public b2Shape
{
public:
	b2CircleShape();

	/// Implement b2Shape.
	b2Shape* Clone(b2BlockAllocator* allocator) const override;

	/// Implement b2Shape.
	bool TestPoint(const b2Transform& transform, const b2Vec2& p) const override;

	// @see b2Shape::ComputeDistance
	void ComputeDistance(const b2Transform& xf, const b2Vec2& p, float32* distance, b2Vec2* normal) const override;

	/// Implement b2Shape.
	/// @note because the circle is solid, rays that start inside do not hit because the normal is
	/// not defined.
	bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
		const b2Transform& transform) const override;

	/// @see b2Shape::ComputeAABB
	void ComputeAABB(b2AABB* aabb, const b2Transform& transform) const override;

	/// @see b2Shape::ComputeMass
	void ComputeMass(b2MassData* massData, float density) const override;

#if LIQUIDFUN_EXTERNAL_LANGUAGE_API
public:
	/// Set position with direct floats.
	void SetPosition(float32 x, float32 y) { m_p.Set(x, y); }

	/// Get x-coordinate of position.
	float32 GetPositionX() const { return m_p.x; }

	/// Get y-coordinate of position.
	float32 GetPositionY() const { return m_p.y; }
#endif // LIQUIDFUN_EXTERNAL_LANGUAGE_API

	/// Position
	b2Vec2 m_p;
};

inline b2CircleShape::b2CircleShape()
{
	m_type = e_circle;
	m_radius = 0.0f;
	m_p.SetZero();
}

#endif
