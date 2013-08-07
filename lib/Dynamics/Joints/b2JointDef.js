function b2JointDef() {
    this.type = b2Joint.e_unknownJoint;
    this.userData = null;
    this.bodyA = null;
    this.bodyB = null;
    this.collideConnected = false;
};
Box2D.Dynamics.Joints.b2JointDef = b2JointDef;
