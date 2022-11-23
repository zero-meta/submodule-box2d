#ifndef B2_FAKE_JOINT_H
#define B2_FAKE_JOINT_H

#include <Box2D/Dynamics/Joints/b2Joint.h>

/// Fake joint definition.
struct b2FakeJointDef : public b2JointDef
{
	b2FakeJointDef()
	{
		type = e_fakeJoint;
	}

	/// Initialize the bodies
	void Initialize(b2Body* bodyA, b2Body* bodyB);
};

/// A weld joint essentially glues two bodies together. A weld joint may
/// distort somewhat because the island constraint solver is approximate.
class b2FakeJoint : public b2Joint
{
public:
	b2Vec2 GetAnchorA() const;
	b2Vec2 GetAnchorB() const;

	b2Vec2 GetReactionForce(float32 inv_dt) const;
	float32 GetReactionTorque(float32 inv_dt) const;
	/// Dump to b2Log
	void Dump();

protected:

	friend class b2Joint;

	b2FakeJoint(const b2FakeJointDef* def);

	void InitVelocityConstraints(const b2SolverData& data);
	void SolveVelocityConstraints(const b2SolverData& data);
	bool SolvePositionConstraints(const b2SolverData& data);
};

#endif
