function b2RevoluteJointDef() {
   Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
   this.localAnchorA = new b2Vec2();
   this.localAnchorB = new b2Vec2();

   this.__super.b2JointDef.call(this);
   this.type = b2Joint.e_revoluteJoint;
   this.localAnchorA.Set(0.0, 0.0);
   this.localAnchorB.Set(0.0, 0.0);
   this.referenceAngle = 0.0;
   this.lowerAngle = 0.0;
   this.upperAngle = 0.0;
   this.maxMotorTorque = 0.0;
   this.motorSpeed = 0.0;
   this.enableLimit = false;
   this.enableMotor = false;

};
Box2D.Dynamics.Joints.b2RevoluteJointDef = b2RevoluteJointDef;
Box2D.inherit(b2RevoluteJointDef, Box2D.Dynamics.Joints.b2JointDef);
b2RevoluteJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;

b2RevoluteJointDef.prototype.Initialize = function(bA, bB, anchor) {
   this.bodyA = bA;
   this.bodyB = bB;
   this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
   this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
   this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
}