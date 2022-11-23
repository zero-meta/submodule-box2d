var b2FakeJointDef_Create = Module.cwrap("b2FakeJointDef_Create",
  'number',
  ['number',
    // joint Def
    'number', 'number', 'number']);

var b2FakeJointDef_InitializeAndCreate = Module.cwrap("b2FakeJointDef_InitializeAndCreate",
  'number',
  ['number',
    // initialize args
    'number', 'number'
    // joint def
    'number']);

/** @constructor */
function b2FakeJointDef() {
  // joint def
  this.bodyA = null;
  this.bodyB = null;
  this.collideConnected = false;
}

b2FakeJointDef.prototype.Create = function(world) {
  var weldJoint = new b2FakeJoint(this);
  weldJoint.ptr = b2FakeJointDef_Create(
    world.ptr,
    // joint def
    this.bodyA.ptr, this.bodyB.ptr, this.collideConnected);
  return weldJoint;
};

b2FakeJointDef.prototype.InitializeAndCreate  = function(bodyA, bodyB, anchor) {
  this.bodyA = bodyA;
  this.bodyB = bodyB;
  var weldJoint = new b2FakeJoint(this);
  weldJoint.ptr = b2FakeJointDef_InitializeAndCreate(
    world.ptr,
    // InitializeArgs
    this.bodyA.ptr, this.bodyB.ptr,
    // joint def
    this.collideConnected);
  b2World._Push(weldJoint, world.joints);
  return weldJoint;
};

/** @constructor */
function b2FakeJoint(def) {
  this.bodyA = def.bodyA;
  this.bodyB = def.bodyB;
  this.next = null;
  this.ptr = null;
}
