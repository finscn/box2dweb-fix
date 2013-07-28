function b2JointDef() {
   b2JointDef.b2JointDef.apply(this, arguments);
   if (this.constructor === b2JointDef) this.b2JointDef.apply(this, arguments);
};
Box2D.Dynamics.Joints.b2JointDef = b2JointDef;
b2JointDef.b2JointDef = function() {};
b2JointDef.prototype.b2JointDef = function() {
   this.type = b2Joint.e_unknownJoint;
   this.userData = null;
   this.bodyA = null;
   this.bodyB = null;
   this.collideConnected = false;
}