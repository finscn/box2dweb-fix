function b2NullContact() {
    Box2D.Dynamics.Contacts.b2Contact.apply(this, arguments);
};
Box2D.Dynamics.Contacts.b2NullContact = b2NullContact;
Box2D.inherit(b2NullContact, Box2D.Dynamics.Contacts.b2Contact);
b2NullContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;

b2NullContact.prototype.Evaluate = function() {}