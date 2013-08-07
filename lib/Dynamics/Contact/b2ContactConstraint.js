function b2ContactConstraint() {
    this.localPlaneNormal = new b2Vec2();
    this.localPoint = new b2Vec2();
    this.normal = new b2Vec2();
    this.normalMass = new b2Mat22();
    this.K = new b2Mat22();
    this.points = new Vector(b2Settings.b2_maxManifoldPoints);
    for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
        this.points[i] = new b2ContactConstraintPoint();
    }
};
Box2D.Dynamics.Contacts.b2ContactConstraint = b2ContactConstraint;