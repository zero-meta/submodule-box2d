#ifndef B2WELDJOINTJSBINDINGS_H
#define B2WELDJOINTJSBINDINGS_H

extern "C" {
void* b2FakeJointDef_Create(
    void* world,
    // joint def
    void* bodyA, void* bodyB, double collideConnected);

void* b2FakeJointDef_InitializeAndCreate(
    void* world,
    // initialize args
    void* bodyA, void* bodyB,
    // joint def
    double collideConnected);
}

#endif
