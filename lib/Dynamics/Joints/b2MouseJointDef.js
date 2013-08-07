function b2MouseJointDef() {
   Box2D.Dynamics.Joints.apply(this, arguments);
   this.target = new b2Vec2();
   // this.__super.constructor.call(this);
   this.type = b2Joint.e_mouseJoint;
   this.maxForce = 0.0;
   this.frequencyHz = 5.0;
   this.dampingRatio = 0.7;
};
Box2D.Dynamics.Joints.b2MouseJointDef = b2MouseJointDef;
Box2D.inherit(b2MouseJointDef, Box2D.Dynamics.Joints.b2JointDef);
b2MouseJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;