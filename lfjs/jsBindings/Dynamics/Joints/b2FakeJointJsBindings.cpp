#include <Box2D/Box2D.h>
void* b2FakeJointDef_Create(
    void* world,
    // joint def
    void* bodyA, void* bodyB, double collideConnected) {
  b2FakeJointDef def;
  def.bodyA = (b2Body*)bodyA;
  def.bodyB = (b2Body*)bodyB;
  def.collideConnected = collideConnected;

  return ((b2World*)world)->CreateJoint(&def);
}

void* b2FakeJointDef_InitializeAndCreate(
    void* world,
    // initialize args
    void* bodyA, void* bodyB,
    // joint def
    double collideConnected) {
  b2FakeJointDef def;
  def.collideConnected = collideConnected;

  def.Initialize((b2Body*)bodyA, (b2Body*)bodyB);

  return ((b2World*)world)->CreateJoint(&def);
}
