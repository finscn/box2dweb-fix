function b2PrismaticJointDef() {
    Box2D.Dynamics.Joints.b2JointDef.apply(this, arguments);
    this.localAnchorA = new b2Vec2();
    this.localAnchorB = new b2Vec2();
    this.localAxisA = new b2Vec2();

    // this.__super.constructor.call(this);
    this.type = b2Joint.e_prismaticJoint;
    this.localAxisA.Set(1.0, 0.0);
    this.referenceAngle = 0.0;
    this.enableLimit = false;
    this.lowerTranslation = 0.0;
    this.upperTranslation = 0.0;
    this.enableMotor = false;
    this.maxMotorForce = 0.0;
    this.motorSpeed = 0.0;

};
Box2D.Dynamics.Joints.b2PrismaticJointDef = b2PrismaticJointDef;
Box2D.inherit(b2PrismaticJointDef, Box2D.Dynamics.Joints.b2JointDef);
b2PrismaticJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;


b2PrismaticJointDef.prototype.Initialize = function(bA, bB, anchor, axis) {
    this.bodyA = bA;
    this.bodyB = bB;
    this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
    this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
    this.localAxisA = this.bodyA.GetLocalVector(axis);
    this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
}