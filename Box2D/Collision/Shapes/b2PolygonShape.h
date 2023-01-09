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

#ifndef B2_POLYGON_SHAPE_H
#define B2_POLYGON_SHAPE_H

#include "Box2D/Collision/Shapes/b2Shape.h"

/// A solid convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.
class B2_API b2PolygonShape : public b2Shape
{
public:
	b2PolygonShape();

	/// Implement b2Shape.
	b2Shape* Clone(b2BlockAllocator* allocator) const override;

	/// Create a convex hull from the given array of local points.
	/// The count must be in the range [3, b2_maxPolygonVertices].
	/// @warning the points may be re-ordered, even if they form a convex polygon
	/// @warning collinear points are handled but not removed. Collinear points
	/// may lead to poor stacking behavior.
	///
	/// CORONASDK/CORONALABS NOTE: We changed the return value from "void" to
	/// "bool" because we want to know when this function fails.
	bool Set(const b2Vec2* points, int32 count);

	/// Build vertices to represent an axis-aligned box centered on the local origin.
	/// @param hx the half-width.
	/// @param hy the half-height.
	void SetAsBox(float hx, float hy);

	/// Build vertices to represent an oriented box.
	/// @param hx the half-width.
	/// @param hy the half-height.
	/// @param center the center of the box in local coordinates.
	/// @param angle the rotation of the box in local coordinates.
	void SetAsBox(float hx, float hy, const b2Vec2& center, float angle);

	/// @see b2Shape::TestPoint
	bool TestPoint(const b2Transform& transform, const b2Vec2& p) const override;

	// @see b2Shape::ComputeDistance
	void ComputeDistance(const b2Transform& xf, const b2Vec2& p, float32* distance, b2Vec2* normal) const override;

	/// Implement b2Shape.
	/// @note because the polygon is solid, rays that start inside do not hit because the normal is
	/// not defined.
	bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
		const b2Transform& transform) const override;

	/// @see b2Shape::ComputeAABB
	void ComputeAABB(b2AABB* aabb, const b2Transform& transform) const override;

	/// @see b2Shape::ComputeMass
	void ComputeMass(b2MassData* massData, float density) const override;

	/// Validate convexity. This is a very time consuming operation.
	/// @returns true if valid
	bool Validate() const;

#if LIQUIDFUN_EXTERNAL_LANGUAGE_API
public:
	/// Set centroid with direct floats.
	void SetCentroid(float32 x, float32 y);

	/// SetAsBox with direct floats for center.
	/// @see b2Shape::SetAsBox
	void SetAsBox(float32 hx,
								float32 hy,
								float32 centerX,
								float32 centerY,
								float32 angle);
#endif // LIQUIDFUN_EXTERNAL_LANGUAGE_API

	b2Vec2 m_centroid;
	b2Vec2 m_vertices[b2_maxPolygonVertices];
	b2Vec2 m_normals[b2_maxPolygonVertices];
	int32 m_count;
};

inline b2PolygonShape::b2PolygonShape()
{
	m_type = e_polygon;
	m_radius = b2_polygonRadius;
	m_count = 0;
	m_centroid.SetZero();
}

#if LIQUIDFUN_EXTERNAL_LANGUAGE_API
inline void b2PolygonShape::SetCentroid(float32 x, float32 y)
{
	m_centroid.Set(x, y);
}

inline void b2PolygonShape::SetAsBox(float32 hx,
									 float32 hy,
									 float32 centerX,
									 float32 centerY,
									 float32 angle) {
	SetAsBox(hx, hy, b2Vec2(centerX, centerY), angle);
}
#endif // LIQUIDFUN_EXTERNAL_LANGUAGE_API

#endif
