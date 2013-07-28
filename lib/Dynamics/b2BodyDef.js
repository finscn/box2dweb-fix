function b2BodyDef() {
   this.position = new b2Vec2();
   this.linearVelocity = new b2Vec2();
   this.userData = null;
   this.position.Set(0.0, 0.0);
   this.angle = 0.0;
   this.linearVelocity.Set(0, 0);
   this.angularVelocity = 0.0;
   this.linearDamping = 0.0;
   this.angularDamping = 0.0;
   this.allowSleep = true;
   this.awake = true;
   this.fixedRotation = false;
   this.bullet = false;
   this.type = b2Body.b2_staticBody;
   this.active = true;
   this.inertiaScale = 1.0;
};
Box2D.Dynamics.b2BodyDef = b2BodyDef;