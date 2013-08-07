function b2RevoluteJoint(def) {
    Box2D.Dynamics.Joints.b2Joint.apply(this, arguments);
    this.K = new b2Mat22();
    this.K1 = new b2Mat22();
    this.K2 = new b2Mat22();
    this.K3 = new b2Mat22();
    this.impulse3 = new b2Vec3();
    this.impulse2 = new b2Vec2();
    this.reduced = new b2Vec2();
    this.m_localAnchor1 = new b2Vec2();
    this.m_localAnchor2 = new b2Vec2();
    this.m_impulse = new b2Vec3();
    this.m_mass = new b2Mat33();
    // this.__super.constructor.call(this, def);
    this.m_localAnchor1.SetV(def.localAnchorA);
    this.m_localAnchor2.SetV(def.localAnchorB);
    this.m_referenceAngle = def.referenceAngle;
    this.m_impulse.SetZero();
    this.m_motorImpulse = 0.0;
    this.m_lowerAngle = def.lowerAngle;
    this.m_upperAngle = def.upperAngle;
    this.m_maxMotorTorque = def.maxMotorTorque;
    this.m_motorSpeed = def.motorSpeed;
    this.m_enableLimit = def.enableLimit;
    this.m_enableMotor = def.enableMotor;
    this.m_limitState = b2Joint.e_inactiveLimit;
};
Box2D.Dynamics.Joints.b2RevoluteJoint = b2RevoluteJoint;
Box2D.inherit(b2RevoluteJoint, Box2D.Dynamics.Joints.b2Joint);
b2RevoluteJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;

b2RevoluteJoint.prototype.GetAnchorA = function() {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
}
b2RevoluteJoint.prototype.GetAnchorB = function() {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
}
b2RevoluteJoint.prototype.GetReactionForce = function(inv_dt) {
    if (inv_dt === undefined) inv_dt = 0;
    return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
}
b2RevoluteJoint.prototype.GetReactionTorque = function(inv_dt) {
    if (inv_dt === undefined) inv_dt = 0;
    return inv_dt * this.m_impulse.z;
}
b2RevoluteJoint.prototype.GetJointAngle = function() {
    return this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a - this.m_referenceAngle;
}
b2RevoluteJoint.prototype.GetJointSpeed = function() {
    return this.m_bodyB.m_angularVelocity - this.m_bodyA.m_angularVelocity;
}
b2RevoluteJoint.prototype.IsLimitEnabled = function() {
    return this.m_enableLimit;
}
b2RevoluteJoint.prototype.EnableLimit = function(flag) {
    this.m_enableLimit = flag;
}
b2RevoluteJoint.prototype.GetLowerLimit = function() {
    return this.m_lowerAngle;
}
b2RevoluteJoint.prototype.GetUpperLimit = function() {
    return this.m_upperAngle;
}
b2RevoluteJoint.prototype.SetLimits = function(lower, upper) {
    if (lower === undefined) lower = 0;
    if (upper === undefined) upper = 0;
    this.m_lowerAngle = lower;
    this.m_upperAngle = upper;
}
b2RevoluteJoint.prototype.IsMotorEnabled = function() {
    this.m_bodyA.SetAwake(true);
    this.m_bodyB.SetAwake(true);
    return this.m_enableMotor;
}
b2RevoluteJoint.prototype.EnableMotor = function(flag) {
    this.m_enableMotor = flag;
}
b2RevoluteJoint.prototype.SetMotorSpeed = function(speed) {
    if (speed === undefined) speed = 0;
    this.m_bodyA.SetAwake(true);
    this.m_bodyB.SetAwake(true);
    this.m_motorSpeed = speed;
}
b2RevoluteJoint.prototype.GetMotorSpeed = function() {
    return this.m_motorSpeed;
}
b2RevoluteJoint.prototype.SetMaxMotorTorque = function(torque) {
    if (torque === undefined) torque = 0;
    this.m_maxMotorTorque = torque;
}
b2RevoluteJoint.prototype.GetMotorTorque = function() {
    return this.m_maxMotorTorque;
}

b2RevoluteJoint.prototype.InitVelocityConstraints = function(step) {
    var bA = this.m_bodyA;
    var bB = this.m_bodyB;
    var tMat;
    var tX = 0;
    if (this.m_enableMotor || this.m_enableLimit) {}
    tMat = bA.m_xf.R;
    var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
    var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
    r1X = tX;
    tMat = bB.m_xf.R;
    var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
    var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
    r2X = tX;
    var m1 = bA.m_invMass;
    var m2 = bB.m_invMass;
    var i1 = bA.m_invI;
    var i2 = bB.m_invI;
    this.m_mass.col1.x = m1 + m2 + r1Y * r1Y * i1 + r2Y * r2Y * i2;
    this.m_mass.col2.x = (-r1Y * r1X * i1) - r2Y * r2X * i2;
    this.m_mass.col3.x = (-r1Y * i1) - r2Y * i2;
    this.m_mass.col1.y = this.m_mass.col2.x;
    this.m_mass.col2.y = m1 + m2 + r1X * r1X * i1 + r2X * r2X * i2;
    this.m_mass.col3.y = r1X * i1 + r2X * i2;
    this.m_mass.col1.z = this.m_mass.col3.x;
    this.m_mass.col2.z = this.m_mass.col3.y;
    this.m_mass.col3.z = i1 + i2;
    this.m_motorMass = 1.0 / (i1 + i2);
    if (this.m_enableMotor == false) {
        this.m_motorImpulse = 0.0;
    }
    if (this.m_enableLimit) {
        var jointAngle = bB.m_sweep.a - bA.m_sweep.a - this.m_referenceAngle;
        if (b2Math.Abs(this.m_upperAngle - this.m_lowerAngle) < 2.0 * b2Settings.b2_angularSlop) {
            this.m_limitState = b2Joint.e_equalLimits;
        } else if (jointAngle <= this.m_lowerAngle) {
            if (this.m_limitState != b2Joint.e_atLowerLimit) {
                this.m_impulse.z = 0.0;
            }
            this.m_limitState = b2Joint.e_atLowerLimit;
        } else if (jointAngle >= this.m_upperAngle) {
            if (this.m_limitState != b2Joint.e_atUpperLimit) {
                this.m_impulse.z = 0.0;
            }
            this.m_limitState = b2Joint.e_atUpperLimit;
        } else {
            this.m_limitState = b2Joint.e_inactiveLimit;
            this.m_impulse.z = 0.0;
        }
    } else {
        this.m_limitState = b2Joint.e_inactiveLimit;
    }
    if (step.warmStarting) {
        this.m_impulse.x *= step.dtRatio;
        this.m_impulse.y *= step.dtRatio;
        this.m_motorImpulse *= step.dtRatio;
        var PX = this.m_impulse.x;
        var PY = this.m_impulse.y;
        bA.m_linearVelocity.x -= m1 * PX;
        bA.m_linearVelocity.y -= m1 * PY;
        bA.m_angularVelocity -= i1 * ((r1X * PY - r1Y * PX) + this.m_motorImpulse + this.m_impulse.z);
        bB.m_linearVelocity.x += m2 * PX;
        bB.m_linearVelocity.y += m2 * PY;
        bB.m_angularVelocity += i2 * ((r2X * PY - r2Y * PX) + this.m_motorImpulse + this.m_impulse.z);
    } else {
        this.m_impulse.SetZero();
        this.m_motorImpulse = 0.0;
    }
}
b2RevoluteJoint.prototype.SolveVelocityConstraints = function(step) {
    var bA = this.m_bodyA;
    var bB = this.m_bodyB;
    var tMat;
    var tX = 0;
    var newImpulse = 0;
    var r1X = 0;
    var r1Y = 0;
    var r2X = 0;
    var r2Y = 0;
    var v1 = bA.m_linearVelocity;
    var w1 = bA.m_angularVelocity;
    var v2 = bB.m_linearVelocity;
    var w2 = bB.m_angularVelocity;
    var m1 = bA.m_invMass;
    var m2 = bB.m_invMass;
    var i1 = bA.m_invI;
    var i2 = bB.m_invI;
    if (this.m_enableMotor && this.m_limitState != b2Joint.e_equalLimits) {
        var Cdot = w2 - w1 - this.m_motorSpeed;
        var impulse = this.m_motorMass * ((-Cdot));
        var oldImpulse = this.m_motorImpulse;
        var maxImpulse = step.dt * this.m_maxMotorTorque;
        this.m_motorImpulse = b2Math.Clamp(this.m_motorImpulse + impulse, (-maxImpulse), maxImpulse);
        impulse = this.m_motorImpulse - oldImpulse;
        w1 -= i1 * impulse;
        w2 += i2 * impulse;
    }
    if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit) {
        tMat = bA.m_xf.R;
        r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
        r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
        tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
        r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
        r1X = tX;
        tMat = bB.m_xf.R;
        r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
        r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
        tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
        r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
        r2X = tX;
        var Cdot1X = v2.x + ((-w2 * r2Y)) - v1.x - ((-w1 * r1Y));
        var Cdot1Y = v2.y + (w2 * r2X) - v1.y - (w1 * r1X);
        var Cdot2 = w2 - w1;
        this.m_mass.Solve33(this.impulse3, (-Cdot1X), (-Cdot1Y), (-Cdot2));
        if (this.m_limitState == b2Joint.e_equalLimits) {
            this.m_impulse.Add(this.impulse3);
        } else if (this.m_limitState == b2Joint.e_atLowerLimit) {
            newImpulse = this.m_impulse.z + this.impulse3.z;
            if (newImpulse < 0.0) {
                this.m_mass.Solve22(this.reduced, (-Cdot1X), (-Cdot1Y));
                this.impulse3.x = this.reduced.x;
                this.impulse3.y = this.reduced.y;
                this.impulse3.z = (-this.m_impulse.z);
                this.m_impulse.x += this.reduced.x;
                this.m_impulse.y += this.reduced.y;
                this.m_impulse.z = 0.0;
            }
        } else if (this.m_limitState == b2Joint.e_atUpperLimit) {
            newImpulse = this.m_impulse.z + this.impulse3.z;
            if (newImpulse > 0.0) {
                this.m_mass.Solve22(this.reduced, (-Cdot1X), (-Cdot1Y));
                this.impulse3.x = this.reduced.x;
                this.impulse3.y = this.reduced.y;
                this.impulse3.z = (-this.m_impulse.z);
                this.m_impulse.x += this.reduced.x;
                this.m_impulse.y += this.reduced.y;
                this.m_impulse.z = 0.0;
            }
        }
        v1.x -= m1 * this.impulse3.x;
        v1.y -= m1 * this.impulse3.y;
        w1 -= i1 * (r1X * this.impulse3.y - r1Y * this.impulse3.x + this.impulse3.z);
        v2.x += m2 * this.impulse3.x;
        v2.y += m2 * this.impulse3.y;
        w2 += i2 * (r2X * this.impulse3.y - r2Y * this.impulse3.x + this.impulse3.z);
    } else {
        tMat = bA.m_xf.R;
        r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
        r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
        tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
        r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
        r1X = tX;
        tMat = bB.m_xf.R;
        r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
        r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
        tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
        r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
        r2X = tX;
        var CdotX = v2.x + ((-w2 * r2Y)) - v1.x - ((-w1 * r1Y));
        var CdotY = v2.y + (w2 * r2X) - v1.y - (w1 * r1X);
        this.m_mass.Solve22(this.impulse2, (-CdotX), (-CdotY));
        this.m_impulse.x += this.impulse2.x;
        this.m_impulse.y += this.impulse2.y;
        v1.x -= m1 * this.impulse2.x;
        v1.y -= m1 * this.impulse2.y;
        w1 -= i1 * (r1X * this.impulse2.y - r1Y * this.impulse2.x);
        v2.x += m2 * this.impulse2.x;
        v2.y += m2 * this.impulse2.y;
        w2 += i2 * (r2X * this.impulse2.y - r2Y * this.impulse2.x);
    }
    bA.m_linearVelocity.SetV(v1);
    bA.m_angularVelocity = w1;
    bB.m_linearVelocity.SetV(v2);
    bB.m_angularVelocity = w2;
}
b2RevoluteJoint.prototype.SolvePositionConstraints = function(baumgarte) {
    if (baumgarte === undefined) baumgarte = 0;
    var oldLimitImpulse = 0;
    var C = 0;
    var tMat;
    var bA = this.m_bodyA;
    var bB = this.m_bodyB;
    var angularError = 0.0;
    var positionError = 0.0;
    var tX = 0;
    var impulseX = 0;
    var impulseY = 0;
    if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit) {
        var angle = bB.m_sweep.a - bA.m_sweep.a - this.m_referenceAngle;
        var limitImpulse = 0.0;
        if (this.m_limitState == b2Joint.e_equalLimits) {
            C = b2Math.Clamp(angle - this.m_lowerAngle, (-b2Settings.b2_maxAngularCorrection), b2Settings.b2_maxAngularCorrection);
            limitImpulse = (-this.m_motorMass * C);
            angularError = b2Math.Abs(C);
        } else if (this.m_limitState == b2Joint.e_atLowerLimit) {
            C = angle - this.m_lowerAngle;
            angularError = (-C);
            C = b2Math.Clamp(C + b2Settings.b2_angularSlop, (-b2Settings.b2_maxAngularCorrection), 0.0);
            limitImpulse = (-this.m_motorMass * C);
        } else if (this.m_limitState == b2Joint.e_atUpperLimit) {
            C = angle - this.m_upperAngle;
            angularError = C;
            C = b2Math.Clamp(C - b2Settings.b2_angularSlop, 0.0, b2Settings.b2_maxAngularCorrection);
            limitImpulse = (-this.m_motorMass * C);
        }
        bA.m_sweep.a -= bA.m_invI * limitImpulse;
        bB.m_sweep.a += bB.m_invI * limitImpulse;
        bA.SynchronizeTransform();
        bB.SynchronizeTransform();
    } {
        tMat = bA.m_xf.R;
        var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
        var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
        tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
        r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
        r1X = tX;
        tMat = bB.m_xf.R;
        var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
        var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
        tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
        r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
        r2X = tX;
        var CX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
        var CY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
        var CLengthSquared = CX * CX + CY * CY;
        var CLength = Math.sqrt(CLengthSquared);
        positionError = CLength;
        var invMass1 = bA.m_invMass;
        var invMass2 = bB.m_invMass;
        var invI1 = bA.m_invI;
        var invI2 = bB.m_invI;
        var k_allowedStretch = 10.0 * b2Settings.b2_linearSlop;
        if (CLengthSquared > k_allowedStretch * k_allowedStretch) {
            var uX = CX / CLength;
            var uY = CY / CLength;
            var k = invMass1 + invMass2;
            var m = 1.0 / k;
            impulseX = m * ((-CX));
            impulseY = m * ((-CY));
            var k_beta = 0.5;
            bA.m_sweep.c.x -= k_beta * invMass1 * impulseX;
            bA.m_sweep.c.y -= k_beta * invMass1 * impulseY;
            bB.m_sweep.c.x += k_beta * invMass2 * impulseX;
            bB.m_sweep.c.y += k_beta * invMass2 * impulseY;
            CX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
            CY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
        }
        this.K1.col1.x = invMass1 + invMass2;
        this.K1.col2.x = 0.0;
        this.K1.col1.y = 0.0;
        this.K1.col2.y = invMass1 + invMass2;
        this.K2.col1.x = invI1 * r1Y * r1Y;
        this.K2.col2.x = (-invI1 * r1X * r1Y);
        this.K2.col1.y = (-invI1 * r1X * r1Y);
        this.K2.col2.y = invI1 * r1X * r1X;
        this.K3.col1.x = invI2 * r2Y * r2Y;
        this.K3.col2.x = (-invI2 * r2X * r2Y);
        this.K3.col1.y = (-invI2 * r2X * r2Y);
        this.K3.col2.y = invI2 * r2X * r2X;
        this.K.SetM(this.K1);
        this.K.AddM(this.K2);
        this.K.AddM(this.K3);
        this.K.Solve(b2RevoluteJoint.tImpulse, (-CX), (-CY));
        impulseX = b2RevoluteJoint.tImpulse.x;
        impulseY = b2RevoluteJoint.tImpulse.y;
        bA.m_sweep.c.x -= bA.m_invMass * impulseX;
        bA.m_sweep.c.y -= bA.m_invMass * impulseY;
        bA.m_sweep.a -= bA.m_invI * (r1X * impulseY - r1Y * impulseX);
        bB.m_sweep.c.x += bB.m_invMass * impulseX;
        bB.m_sweep.c.y += bB.m_invMass * impulseY;
        bB.m_sweep.a += bB.m_invI * (r2X * impulseY - r2Y * impulseX);
        bA.SynchronizeTransform();
        bB.SynchronizeTransform();
    }
    return positionError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
}
Box2D.postDefs.push(function() {
    Box2D.Dynamics.Joints.b2RevoluteJoint.tImpulse = new b2Vec2();
});