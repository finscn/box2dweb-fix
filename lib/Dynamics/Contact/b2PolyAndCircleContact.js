function b2PolyAndCircleContact() {
   Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);

};
Box2D.Dynamics.Contacts.b2PolyAndCircleContact = b2PolyAndCircleContact;
Box2D.inherit(b2PolyAndCircleContact, Box2D.Dynamics.Contacts.b2Contact);
b2PolyAndCircleContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;

b2PolyAndCircleContact.Create = function(allocator) {
   return new b2PolyAndCircleContact();
}
b2PolyAndCircleContact.Destroy = function(contact, allocator) {}
b2PolyAndCircleContact.prototype.Reset = function(fixtureA, fixtureB) {
   this.__super.Reset.call(this, fixtureA, fixtureB);
   b2Settings.b2Assert(fixtureA.GetType() == b2Shape.e_polygonShape);
   b2Settings.b2Assert(fixtureB.GetType() == b2Shape.e_circleShape);
}
b2PolyAndCircleContact.prototype.Evaluate = function() {
   var bA = this.m_fixtureA.m_body;
   var bB = this.m_fixtureB.m_body;
   b2Collision.CollidePolygonAndCircle(this.m_manifold, (this.m_fixtureA.GetShape() instanceof b2PolygonShape ? this.m_fixtureA.GetShape() : null), bA.m_xf, (this.m_fixtureB.GetShape() instanceof b2CircleShape ? this.m_fixtureB.GetShape() : null), bB.m_xf);
}