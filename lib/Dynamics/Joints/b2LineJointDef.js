function b2LineJointDef() {
    Box2D.Dynamics.Joints.b2JointDef.apply(this, arguments);
    this.localAnchorA = new b2Vec2();
    this.localAnchorB = new b2Vec2();
    this.localAxisA = new b2Vec2();
    // this.__super.constructor.call(this);
    this.type = b2Joint.e_lineJoint;
    this.localAxisA.Set(1.0, 0.0);
    this.enableLimit = false;
    this.lowerTranslation = 0.0;
    this.upperTranslation = 0.0;
    this.enableMotor = false;
    this.maxMotorForce = 0.0;
    this.motorSpeed = 0.0;
};
Box2D.Dynamics.Joints.b2LineJointDef = b2LineJointDef;
Box2D.inherit(b2LineJointDef, Box2D.Dynamics.Joints.b2JointDef);
b2LineJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;

b2LineJointDef.prototype.Initialize = function(bA, bB, anchor, axis) {
    this.bodyA = bA;
    this.bodyB = bB;
    this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
    this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
    this.localAxisA = this.bodyA.GetLocalVector(axis);
}