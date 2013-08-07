function b2DistanceJointDef() {
    Box2D.Dynamics.Joints.b2JointDef.apply(this, arguments);
    this.localAnchorA = new b2Vec2();
    this.localAnchorB = new b2Vec2();
    // this.__super.constructor.call(this);
    this.type = b2Joint.e_distanceJoint;
    this.length = 1.0;
    this.frequencyHz = 0.0;
    this.dampingRatio = 0.0;

};
Box2D.Dynamics.Joints.b2DistanceJointDef = b2DistanceJointDef;
Box2D.inherit(b2DistanceJointDef, Box2D.Dynamics.Joints.b2JointDef);
b2DistanceJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;

b2DistanceJointDef.prototype.Initialize = function(bA, bB, anchorA, anchorB) {
    this.bodyA = bA;
    this.bodyB = bB;
    this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchorA));
    this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchorB));
    var dX = anchorB.x - anchorA.x;
    var dY = anchorB.y - anchorA.y;
    this.length = Math.sqrt(dX * dX + dY * dY);
    this.frequencyHz = 0.0;
    this.dampingRatio = 0.0;
}