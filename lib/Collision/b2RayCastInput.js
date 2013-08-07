function b2RayCastInput(p1, p2, maxFraction) {
    this.p1 = new b2Vec2();
    this.p2 = new b2Vec2();
    if (p1 === undefined) p1 = null;
    if (p2 === undefined) p2 = null;
    if (maxFraction === undefined) maxFraction = 1;
    if (p1) this.p1.SetV(p1);
    if (p2) this.p2.SetV(p2);
    this.maxFraction = maxFraction;
}
Box2D.Collision.b2RayCastInput = b2RayCastInput;