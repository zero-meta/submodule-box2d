/*
* Copyright (c) 2011 Erin Catto http://www.box2d.org
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

#ifndef B2_ROPE_H
#define B2_ROPE_H

#include "Box2D/Common/b2Math.h"

class b2Draw;
struct b2RopeStretch;
struct b2RopeBend;

enum b2StretchingModel
{
	b2_pbdStretchingModel,
	b2_xpbdStretchingModel
};

enum b2BendingModel
{
	b2_springAngleBendingModel = 0,
	b2_pbdAngleBendingModel,
	b2_xpbdAngleBendingModel,
	b2_pbdDistanceBendingModel,
	b2_pbdHeightBendingModel,
	b2_pbdTriangleBendingModel
};

///
struct B2_API b2RopeTuning
{
	b2RopeTuning()
	{
		stretchingModel = b2_pbdStretchingModel;
		bendingModel = b2_pbdAngleBendingModel;
		damping = 0.0f;
		stretchStiffness = 1.0f;
		bendStiffness = 0.5f;
		bendHertz = 1.0f;
		bendDamping = 0.0f;
		isometric = false;
		fixedEffectiveMass = false;
		warmStart = false;
	}

	b2StretchingModel stretchingModel;
	b2BendingModel bendingModel;
	float damping;
	float stretchStiffness;
	float stretchHertz;
	float stretchDamping;
	float bendStiffness;
	float bendHertz;
	float bendDamping;
	bool isometric;
	bool fixedEffectiveMass;
	bool warmStart;
};

///
struct B2_API b2RopeDef
{
	b2RopeDef()
	{
		position.SetZero();
		vertices = nullptr;
		count = 0;
		masses = nullptr;
		gravity.SetZero();
	}

	b2Vec2 position;
	b2Vec2* vertices;
	int32 count;
	float* masses;
	b2Vec2 gravity;
	b2RopeTuning tuning;
};

///
class B2_API b2Rope
{
public:
	b2Rope();
	~b2Rope();

	///
	void Create(const b2RopeDef& def);

	///
	void SetTuning(const b2RopeTuning& tuning);

	///
	void Step(float timeStep, int32 iterations, const b2Vec2& position);

	///
	void Reset(const b2Vec2& position);

	///
	void Draw(b2Draw* draw) const;

private:

	void SolveStretch_PBD();
	void SolveStretch_XPBD(float dt);
	void SolveBend_PBD_Angle();
	void SolveBend_XPBD_Angle(float dt);
	void SolveBend_PBD_Distance();
	void SolveBend_PBD_Height();
	void SolveBend_PBD_Triangle();
	void ApplyBendForces(float dt);

	b2Vec2 m_position;

	int32 m_count;
	int32 m_stretchCount;
	int32 m_bendCount;

	b2RopeStretch* m_stretchConstraints;
	b2RopeBend* m_bendConstraints;

	b2Vec2* m_bindPositions;
	b2Vec2* m_ps;
	b2Vec2* m_p0s;
	b2Vec2* m_vs;

	float* m_invMasses;
	b2Vec2 m_gravity;

	b2RopeTuning m_tuning;
};

#endif
