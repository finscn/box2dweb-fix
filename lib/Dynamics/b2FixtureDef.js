function b2FixtureDef() {
    this.filter = new b2FilterData();
    this.shape = null;
    this.userData = null;
    this.friction = 0.2;
    this.restitution = 0.0;
    this.density = 0.0;
    this.filter.categoryBits = 0x0001;
    this.filter.maskBits = 0xFFFF;
    this.filter.groupIndex = 0;
    this.isSensor = false;

};
Box2D.Dynamics.b2FixtureDef = b2FixtureDef;