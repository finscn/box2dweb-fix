function b2CircleContact() {

    Box2D.Dynamics.Contacts.b2Contact.apply(this, arguments);
};
Box2D.Dynamics.Contacts.b2CircleContact = b2CircleContact;
Box2D.inherit(b2CircleContact, Box2D.Dynamics.Contacts.b2Contact);
b2CircleContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;

b2CircleContact.Create = function(allocator) {
    return new b2CircleContact();
}
b2CircleContact.Destroy = function(contact, allocator) {}
b2CircleContact.prototype.Reset = function(fixtureA, fixtureB) {
    this.__super.Reset.call(this, fixtureA, fixtureB);
}
b2CircleContact.prototype.Evaluate = function() {
    var bA = this.m_fixtureA.GetBody();
    var bB = this.m_fixtureB.GetBody();
    b2Collision.CollideCircles(this.m_manifold, (this.m_fixtureA.GetShape() instanceof b2CircleShape ? this.m_fixtureA.GetShape() : null), bA.m_xf, (this.m_fixtureB.GetShape() instanceof b2CircleShape ? this.m_fixtureB.GetShape() : null), bB.m_xf);
}