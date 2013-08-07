function b2ContactFilter() {

};
Box2D.Dynamics.b2ContactFilter = b2ContactFilter;

b2ContactFilter.prototype.ShouldCollide = function(fixtureA, fixtureB) {
    var filter1 = fixtureA.GetFilterData();
    var filter2 = fixtureB.GetFilterData();
    if (filter1.groupIndex == filter2.groupIndex && filter1.groupIndex != 0) {
        return filter1.groupIndex > 0;
    }
    var collide = (filter1.maskBits & filter2.categoryBits) != 0 && (filter1.categoryBits & filter2.maskBits) != 0;
    return collide;
}
b2ContactFilter.prototype.RayCollide = function(userData, fixture) {
    if (!userData) return true;
    return this.ShouldCollide((userData instanceof b2Fixture ? userData : null), fixture);
}
Box2D.postDefs.push(function() {
    Box2D.Dynamics.b2ContactFilter.b2_defaultFilter = new b2ContactFilter();
});