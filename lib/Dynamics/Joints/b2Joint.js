function b2Joint(def) {
    this.m_edgeA = new b2jointedge();
    this.m_edgeB = new b2JointEdge();
    this.m_localCenterA = new b2Vec2();
    this.m_localCenterB = new b2Vec2();

    b2Settings.b2Assert(def.bodyA != def.bodyB);
    this.m_type = def.type;
    this.m_prev = null;
    this.m_next = null;
    this.m_bodyA = def.bodyA;
    this.m_bodyB = def.bodyB;
    this.m_collideConnected = def.collideConnected;
    this.m_islandFlag = false;
    this.m_userData = def.userData;

};
Box2D.Dynamics.Joints.b2Joint = b2Joint;
b2Joint.prototype.GetType = function() {
    return this.m_type;
}
b2Joint.prototype.GetAnchorA = function() {
    return null;
}
b2Joint.prototype.GetAnchorB = function() {
    return null;
}
b2Joint.prototype.GetReactionForce = function(inv_dt) {
    if (inv_dt === undefined) inv_dt = 0;
    return null;
}
b2Joint.prototype.GetReactionTorque = function(inv_dt) {
    if (inv_dt === undefined) inv_dt = 0;
    return 0.0;
}
b2Joint.prototype.GetBodyA = function() {
    return this.m_bodyA;
}
b2Joint.prototype.GetBodyB = function() {
    return this.m_bodyB;
}
b2Joint.prototype.GetNext = function() {
    return this.m_next;
}
b2Joint.prototype.GetUserData = function() {
    return this.m_userData;
}
b2Joint.prototype.SetUserData = function(data) {
    this.m_userData = data;
}
b2Joint.prototype.IsActive = function() {
    return this.m_bodyA.IsActive() && this.m_bodyB.IsActive();
}
b2Joint.Create = function(def, allocator) {
    var joint = null;
    switch (def.type) {
        case b2Joint.e_distanceJoint:
            {
                joint = new b2DistanceJoint((def instanceof b2DistanceJointDef ? def : null));
            }
            break;
        case b2Joint.e_mouseJoint:
            {
                joint = new b2MouseJoint((def instanceof b2MouseJointDef ? def : null));
            }
            break;
        case b2Joint.e_prismaticJoint:
            {
                joint = new b2PrismaticJoint((def instanceof b2PrismaticJointDef ? def : null));
            }
            break;
        case b2Joint.e_revoluteJoint:
            {
                joint = new b2RevoluteJoint((def instanceof b2RevoluteJointDef ? def : null));
            }
            break;
        case b2Joint.e_pulleyJoint:
            {
                joint = new b2PulleyJoint((def instanceof b2PulleyJointDef ? def : null));
            }
            break;
        case b2Joint.e_gearJoint:
            {
                joint = new b2GearJoint((def instanceof b2GearJointDef ? def : null));
            }
            break;
        case b2Joint.e_lineJoint:
            {
                joint = new b2LineJoint((def instanceof b2LineJointDef ? def : null));
            }
            break;
        case b2Joint.e_weldJoint:
            {
                joint = new b2WeldJoint((def instanceof b2WeldJointDef ? def : null));
            }
            break;
        case b2Joint.e_frictionJoint:
            {
                joint = new b2FrictionJoint((def instanceof b2FrictionJointDef ? def : null));
            }
            break;
        default:
            break;
    }
    return joint;
}
b2Joint.Destroy = function(joint, allocator) {}

b2Joint.prototype.InitVelocityConstraints = function(step) {}
b2Joint.prototype.SolveVelocityConstraints = function(step) {}
b2Joint.prototype.FinalizeVelocityConstraints = function() {}
b2Joint.prototype.SolvePositionConstraints = function(baumgarte) {
    if (baumgarte === undefined) baumgarte = 0;
    return false;
}
Box2D.postDefs.push(function() {
    Box2D.Dynamics.Joints.b2Joint.e_unknownJoint = 0;
    Box2D.Dynamics.Joints.b2Joint.e_revoluteJoint = 1;
    Box2D.Dynamics.Joints.b2Joint.e_prismaticJoint = 2;
    Box2D.Dynamics.Joints.b2Joint.e_distanceJoint = 3;
    Box2D.Dynamics.Joints.b2Joint.e_pulleyJoint = 4;
    Box2D.Dynamics.Joints.b2Joint.e_mouseJoint = 5;
    Box2D.Dynamics.Joints.b2Joint.e_gearJoint = 6;
    Box2D.Dynamics.Joints.b2Joint.e_lineJoint = 7;
    Box2D.Dynamics.Joints.b2Joint.e_weldJoint = 8;
    Box2D.Dynamics.Joints.b2Joint.e_frictionJoint = 9;
    Box2D.Dynamics.Joints.b2Joint.e_inactiveLimit = 0;
    Box2D.Dynamics.Joints.b2Joint.e_atLowerLimit = 1;
    Box2D.Dynamics.Joints.b2Joint.e_atUpperLimit = 2;
    Box2D.Dynamics.Joints.b2Joint.e_equalLimits = 3;
});