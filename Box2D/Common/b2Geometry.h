// TODO License

#ifndef B2_GEOMETRY_H
#define B2_GEOMETRY_H

#include "Box2D/Dynamics/b2Body.h"
#include "Box2D/Dynamics/b2Fixture.h"

void b2CreateLoop(b2Body* b, b2FixtureDef* fd, const b2Vec2* vertices, int32 count, int32 &fixtureIndex);
void b2CreateLoop(b2Body* b, const b2Vec2* vertices, int32 count, int32 &fixtureIndex);
void b2CreateChain(b2Body* b, b2FixtureDef* fd, const b2Vec2* vertices, int32 count, const b2Vec2 prevVertex, const b2Vec2 nextVertex, int32 &fixtureIndex);
void b2CreateChainTwoSided(b2Body* b, b2FixtureDef* fd, const b2Vec2* vertices, int32 count, int32 &fixtureIndex);
void b2CreateChain(b2Body* b, const b2Vec2* vertices, int32 count, const b2Vec2 prevVertex, const b2Vec2 nextVertex, int32 &fixtureIndex);
void b2CreateChainTwoSided(b2Body* b, const b2Vec2* vertices, int32 count);

#endif
