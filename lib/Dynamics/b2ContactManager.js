function b2ContactManager() {
    this.m_world = null;
    this.m_contactCount = 0;
    this.m_contactFilter = b2ContactFilter.b2_defaultFilter;
    this.m_contactListener = b2ContactListener.b2_defaultListener;
    this.m_contactFactory = new b2ContactFactory(this.m_allocator);
    this.m_broadPhase = new b2DynamicTreeBroadPhase();

};
Box2D.Dynamics.b2ContactManager = b2ContactManager;

b2ContactManager.prototype.AddPair = function(proxyUserDataA, proxyUserDataB) {
    var fixtureA = (proxyUserDataA instanceof b2Fixture ? proxyUserDataA : null);
    var fixtureB = (proxyUserDataB instanceof b2Fixture ? proxyUserDataB : null);
    var bodyA = fixtureA.GetBody();
    var bodyB = fixtureB.GetBody();
    if (bodyA == bodyB) return;
    var edge = bodyB.GetContactList();
    while (edge) {
        if (edge.other == bodyA) {
            var fA = edge.contact.GetFixtureA();
            var fB = edge.contact.GetFixtureB();
            if (fA == fixtureA && fB == fixtureB) return;
            if (fA == fixtureB && fB == fixtureA) return;
        }
        edge = edge.next;
    }
    if (bodyB.ShouldCollide(bodyA) == false) {
        return;
    }
    if (this.m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false) {
        return;
    }
    var c = this.m_contactFactory.Create(fixtureA, fixtureB);
    fixtureA = c.GetFixtureA();
    fixtureB = c.GetFixtureB();
    bodyA = fixtureA.m_body;
    bodyB = fixtureB.m_body;
    c.m_prev = null;
    c.m_next = this.m_world.m_contactList;
    if (this.m_world.m_contactList != null) {
        this.m_world.m_contactList.m_prev = c;
    }
    this.m_world.m_contactList = c;
    c.m_nodeA.contact = c;
    c.m_nodeA.other = bodyB;
    c.m_nodeA.prev = null;
    c.m_nodeA.next = bodyA.m_contactList;
    if (bodyA.m_contactList != null) {
        bodyA.m_contactList.prev = c.m_nodeA;
    }
    bodyA.m_contactList = c.m_nodeA;
    c.m_nodeB.contact = c;
    c.m_nodeB.other = bodyA;
    c.m_nodeB.prev = null;
    c.m_nodeB.next = bodyB.m_contactList;
    if (bodyB.m_contactList != null) {
        bodyB.m_contactList.prev = c.m_nodeB;
    }
    bodyB.m_contactList = c.m_nodeB;
    ++this.m_world.m_contactCount;
    return;
}
b2ContactManager.prototype.FindNewContacts = function() {
    this.m_broadPhase.UpdatePairs(Box2D.generateCallback(this, this.AddPair));
}
b2ContactManager.prototype.Destroy = function(c) {
    var fixtureA = c.GetFixtureA();
    var fixtureB = c.GetFixtureB();
    var bodyA = fixtureA.GetBody();
    var bodyB = fixtureB.GetBody();
    if (c.IsTouching()) {
        this.m_contactListener.EndContact(c);
    }
    if (c.m_prev) {
        c.m_prev.m_next = c.m_next;
    }
    if (c.m_next) {
        c.m_next.m_prev = c.m_prev;
    }
    if (c == this.m_world.m_contactList) {
        this.m_world.m_contactList = c.m_next;
    }
    if (c.m_nodeA.prev) {
        c.m_nodeA.prev.next = c.m_nodeA.next;
    }
    if (c.m_nodeA.next) {
        c.m_nodeA.next.prev = c.m_nodeA.prev;
    }
    if (c.m_nodeA == bodyA.m_contactList) {
        bodyA.m_contactList = c.m_nodeA.next;
    }
    if (c.m_nodeB.prev) {
        c.m_nodeB.prev.next = c.m_nodeB.next;
    }
    if (c.m_nodeB.next) {
        c.m_nodeB.next.prev = c.m_nodeB.prev;
    }
    if (c.m_nodeB == bodyB.m_contactList) {
        bodyB.m_contactList = c.m_nodeB.next;
    }
    this.m_contactFactory.Destroy(c);
    --this.m_contactCount;
}
b2ContactManager.prototype.Collide = function() {
    var c = this.m_world.m_contactList;
    while (c) {
        var fixtureA = c.GetFixtureA();
        var fixtureB = c.GetFixtureB();
        var bodyA = fixtureA.GetBody();
        var bodyB = fixtureB.GetBody();
        if (bodyA.IsAwake() == false && bodyB.IsAwake() == false) {
            c = c.GetNext();
            continue;
        }
        if (c.m_flags & b2Contact.e_filterFlag) {
            if (bodyB.ShouldCollide(bodyA) == false) {
                var cNuke = c;
                c = cNuke.GetNext();
                this.Destroy(cNuke);
                continue;
            }
            if (this.m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false) {
                cNuke = c;
                c = cNuke.GetNext();
                this.Destroy(cNuke);
                continue;
            }
            c.m_flags &= ~b2Contact.e_filterFlag;
        }
        var proxyA = fixtureA.m_proxy;
        var proxyB = fixtureB.m_proxy;
        var overlap = this.m_broadPhase.TestOverlap(proxyA, proxyB);
        if (overlap == false) {
            cNuke = c;
            c = cNuke.GetNext();
            this.Destroy(cNuke);
            continue;
        }
        c.Update(this.m_contactListener);
        c = c.GetNext();
    }
}
Box2D.postDefs.push(function() {
    Box2D.Dynamics.b2ContactManager.s_evalCP = new b2ContactPoint();
});