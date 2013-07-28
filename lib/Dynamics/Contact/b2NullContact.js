function b2NullContact() {
   Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
   this.__super.b2Contact.call(this);

};
Box2D.Dynamics.Contacts.b2NullContact = b2NullContact;
Box2D.inherit(b2NullContact, Box2D.Dynamics.Contacts.b2Contact);
b2NullContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;

b2NullContact.prototype.Evaluate = function() {}