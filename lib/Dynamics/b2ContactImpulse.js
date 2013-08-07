function b2ContactImpulse() {
    this.normalImpulses = new Vector_a2j_Number(b2Settings.b2_maxManifoldPoints);
    this.tangentImpulses = new Vector_a2j_Number(b2Settings.b2_maxManifoldPoints);

};
Box2D.Dynamics.b2ContactImpulse = b2ContactImpulse;