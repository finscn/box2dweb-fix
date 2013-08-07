function b2ContactPoint() {
    this.position = new b2Vec2();
    this.velocity = new b2Vec2();
    this.normal = new b2Vec2();
    this.id = new b2ContactID();
};
Box2D.Collision.b2ContactPoint = b2ContactPoint;