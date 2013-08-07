function b2GearJointDef() {
    Box2D.Dynamics.Joints.b2JointDef.apply(this, arguments);
    // this.__super.constructor.call(this);
    this.type = b2Joint.e_gearJoint;
    this.joint1 = null;
    this.joint2 = null;
    this.ratio = 1.0;

};
Box2D.Dynamics.Joints.b2GearJointDef = b2GearJointDef;
Box2D.inherit(b2GearJointDef, Box2D.Dynamics.Joints.b2JointDef);
b2GearJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;