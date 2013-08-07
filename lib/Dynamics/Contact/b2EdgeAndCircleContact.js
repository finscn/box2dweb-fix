function b2EdgeAndCircleContact() {
    Box2D.Dynamics.Contacts.b2Contact.apply(this, arguments);

};
Box2D.Dynamics.Contacts.b2EdgeAndCircleContact = b2EdgeAndCircleContact;
Box2D.inherit(b2EdgeAndCircleContact, Box2D.Dynamics.Contacts.b2Contact);
b2EdgeAndCircleContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;

b2EdgeAndCircleContact.Create = function(allocator) {
    return new b2EdgeAndCircleContact();
}
b2EdgeAndCircleContact.Destroy = function(contact, allocator) {}
b2EdgeAndCircleContact.prototype.Reset = function(fixtureA, fixtureB) {
    this.__super.Reset.call(this, fixtureA, fixtureB);
}
b2EdgeAndCircleContact.prototype.Evaluate = function() {
    var bA = this.m_fixtureA.GetBody();
    var bB = this.m_fixtureB.GetBody();
    this.b2CollideEdgeAndCircle(this.m_manifold, (this.m_fixtureA.GetShape() instanceof b2EdgeShape ? this.m_fixtureA.GetShape() : null), bA.m_xf, (this.m_fixtureB.GetShape() instanceof b2CircleShape ? this.m_fixtureB.GetShape() : null), bB.m_xf);
}
b2EdgeAndCircleContact.prototype.b2CollideEdgeAndCircle = function(manifold, edge, xf1, circle, xf2) {}