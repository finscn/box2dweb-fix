function b2GearJoint(def) {
    Box2D.Dynamics.Joints.b2Joint.apply(this, arguments);
    this.m_groundAnchor1 = new b2Vec2();
    this.m_groundAnchor2 = new b2Vec2();
    this.m_localAnchor1 = new b2Vec2();
    this.m_localAnchor2 = new b2Vec2();
    this.m_J = new b2Jacobian();

    // this.__super.constructor.call(this, def);
    var type1 = def.joint1.m_type;
    var type2 = def.joint2.m_type;
    this.m_revolute1 = null;
    this.m_prismatic1 = null;
    this.m_revolute2 = null;
    this.m_prismatic2 = null;
    var coordinate1 = 0;
    var coordinate2 = 0;
    this.m_ground1 = def.joint1.GetBodyA();
    this.m_bodyA = def.joint1.GetBodyB();
    if (type1 == b2Joint.e_revoluteJoint) {
        this.m_revolute1 = (def.joint1 instanceof b2RevoluteJoint ? def.joint1 : null);
        this.m_groundAnchor1.SetV(this.m_revolute1.m_localAnchor1);
        this.m_localAnchor1.SetV(this.m_revolute1.m_localAnchor2);
        coordinate1 = this.m_revolute1.GetJointAngle();
    } else {
        this.m_prismatic1 = (def.joint1 instanceof b2PrismaticJoint ? def.joint1 : null);
        this.m_groundAnchor1.SetV(this.m_prismatic1.m_localAnchor1);
        this.m_localAnchor1.SetV(this.m_prismatic1.m_localAnchor2);
        coordinate1 = this.m_prismatic1.GetJointTranslation();
    }
    this.m_ground2 = def.joint2.GetBodyA();
    this.m_bodyB = def.joint2.GetBodyB();
    if (type2 == b2Joint.e_revoluteJoint) {
        this.m_revolute2 = (def.joint2 instanceof b2RevoluteJoint ? def.joint2 : null);
        this.m_groundAnchor2.SetV(this.m_revolute2.m_localAnchor1);
        this.m_localAnchor2.SetV(this.m_revolute2.m_localAnchor2);
        coordinate2 = this.m_revolute2.GetJointAngle();
    } else {
        this.m_prismatic2 = (def.joint2 instanceof b2PrismaticJoint ? def.joint2 : null);
        this.m_groundAnchor2.SetV(this.m_prismatic2.m_localAnchor1);
        this.m_localAnchor2.SetV(this.m_prismatic2.m_localAnchor2);
        coordinate2 = this.m_prismatic2.GetJointTranslation();
    }
    this.m_ratio = def.ratio;
    this.m_constant = coordinate1 + this.m_ratio * coordinate2;
    this.m_impulse = 0.0;
};
Box2D.Dynamics.Joints.b2GearJoint = b2GearJoint;
Box2D.inherit(b2GearJoint, Box2D.Dynamics.Joints.b2Joint);
b2GearJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;

b2GearJoint.prototype.GetAnchorA = function() {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
}
b2GearJoint.prototype.GetAnchorB = function() {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
}
b2GearJoint.prototype.GetReactionForce = function(inv_dt) {
    if (inv_dt === undefined) inv_dt = 0;
    return new b2Vec2(inv_dt * this.m_impulse * this.m_J.linearB.x, inv_dt * this.m_impulse * this.m_J.linearB.y);
}
b2GearJoint.prototype.GetReactionTorque = function(inv_dt) {
    if (inv_dt === undefined) inv_dt = 0;
    var tMat = this.m_bodyB.m_xf.R;
    var rX = this.m_localAnchor1.x - this.m_bodyB.m_sweep.localCenter.x;
    var rY = this.m_localAnchor1.y - this.m_bodyB.m_sweep.localCenter.y;
    var tX = tMat.col1.x * rX + tMat.col2.x * rY;
    rY = tMat.col1.y * rX + tMat.col2.y * rY;
    rX = tX;
    var PX = this.m_impulse * this.m_J.linearB.x;
    var PY = this.m_impulse * this.m_J.linearB.y;
    return inv_dt * (this.m_impulse * this.m_J.angularB - rX * PY + rY * PX);
}
b2GearJoint.prototype.GetRatio = function() {
    return this.m_ratio;
}
b2GearJoint.prototype.SetRatio = function(ratio) {
    if (ratio === undefined) ratio = 0;
    this.m_ratio = ratio;
}


b2GearJoint.prototype.InitVelocityConstraints = function(step) {
    var g1 = this.m_ground1;
    var g2 = this.m_ground2;
    var bA = this.m_bodyA;
    var bB = this.m_bodyB;
    var ugX = 0;
    var ugY = 0;
    var rX = 0;
    var rY = 0;
    var tMat;
    var tVec;
    var crug = 0;
    var tX = 0;
    var K = 0.0;
    this.m_J.SetZero();
    if (this.m_revolute1) {
        this.m_J.angularA = (-1.0);
        K += bA.m_invI;
    } else {
        tMat = g1.m_xf.R;
        tVec = this.m_prismatic1.m_localXAxis1;
        ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tMat = bA.m_xf.R;
        rX = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
        rY = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
        tX = tMat.col1.x * rX + tMat.col2.x * rY;
        rY = tMat.col1.y * rX + tMat.col2.y * rY;
        rX = tX;
        crug = rX * ugY - rY * ugX;
        this.m_J.linearA.Set((-ugX), (-ugY));
        this.m_J.angularA = (-crug);
        K += bA.m_invMass + bA.m_invI * crug * crug;
    }
    if (this.m_revolute2) {
        this.m_J.angularB = (-this.m_ratio);
        K += this.m_ratio * this.m_ratio * bB.m_invI;
    } else {
        tMat = g2.m_xf.R;
        tVec = this.m_prismatic2.m_localXAxis1;
        ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tMat = bB.m_xf.R;
        rX = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
        rY = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
        tX = tMat.col1.x * rX + tMat.col2.x * rY;
        rY = tMat.col1.y * rX + tMat.col2.y * rY;
        rX = tX;
        crug = rX * ugY - rY * ugX;
        this.m_J.linearB.Set((-this.m_ratio * ugX), (-this.m_ratio * ugY));
        this.m_J.angularB = (-this.m_ratio * crug);
        K += this.m_ratio * this.m_ratio * (bB.m_invMass + bB.m_invI * crug * crug);
    }
    this.m_mass = K > 0.0 ? 1.0 / K : 0.0;
    if (step.warmStarting) {
        bA.m_linearVelocity.x += bA.m_invMass * this.m_impulse * this.m_J.linearA.x;
        bA.m_linearVelocity.y += bA.m_invMass * this.m_impulse * this.m_J.linearA.y;
        bA.m_angularVelocity += bA.m_invI * this.m_impulse * this.m_J.angularA;
        bB.m_linearVelocity.x += bB.m_invMass * this.m_impulse * this.m_J.linearB.x;
        bB.m_linearVelocity.y += bB.m_invMass * this.m_impulse * this.m_J.linearB.y;
        bB.m_angularVelocity += bB.m_invI * this.m_impulse * this.m_J.angularB;
    } else {
        this.m_impulse = 0.0;
    }
}
b2GearJoint.prototype.SolveVelocityConstraints = function(step) {
    var bA = this.m_bodyA;
    var bB = this.m_bodyB;
    var Cdot = this.m_J.Compute(bA.m_linearVelocity, bA.m_angularVelocity, bB.m_linearVelocity, bB.m_angularVelocity);
    var impulse = (-this.m_mass * Cdot);
    this.m_impulse += impulse;
    bA.m_linearVelocity.x += bA.m_invMass * impulse * this.m_J.linearA.x;
    bA.m_linearVelocity.y += bA.m_invMass * impulse * this.m_J.linearA.y;
    bA.m_angularVelocity += bA.m_invI * impulse * this.m_J.angularA;
    bB.m_linearVelocity.x += bB.m_invMass * impulse * this.m_J.linearB.x;
    bB.m_linearVelocity.y += bB.m_invMass * impulse * this.m_J.linearB.y;
    bB.m_angularVelocity += bB.m_invI * impulse * this.m_J.angularB;
}
b2GearJoint.prototype.SolvePositionConstraints = function(baumgarte) {
    if (baumgarte === undefined) baumgarte = 0;
    var linearError = 0.0;
    var bA = this.m_bodyA;
    var bB = this.m_bodyB;
    var coordinate1 = 0;
    var coordinate2 = 0;
    if (this.m_revolute1) {
        coordinate1 = this.m_revolute1.GetJointAngle();
    } else {
        coordinate1 = this.m_prismatic1.GetJointTranslation();
    }
    if (this.m_revolute2) {
        coordinate2 = this.m_revolute2.GetJointAngle();
    } else {
        coordinate2 = this.m_prismatic2.GetJointTranslation();
    }
    var C = this.m_constant - (coordinate1 + this.m_ratio * coordinate2);
    var impulse = (-this.m_mass * C);
    bA.m_sweep.c.x += bA.m_invMass * impulse * this.m_J.linearA.x;
    bA.m_sweep.c.y += bA.m_invMass * impulse * this.m_J.linearA.y;
    bA.m_sweep.a += bA.m_invI * impulse * this.m_J.angularA;
    bB.m_sweep.c.x += bB.m_invMass * impulse * this.m_J.linearB.x;
    bB.m_sweep.c.y += bB.m_invMass * impulse * this.m_J.linearB.y;
    bB.m_sweep.a += bB.m_invI * impulse * this.m_J.angularB;
    bA.SynchronizeTransform();
    bB.SynchronizeTransform();
    return linearError < b2Settings.b2_linearSlop;
}