function b2Shape() {
   this.m_type = b2Shape.e_unknownShape;
   this.m_radius = b2Settings.b2_linearSlop;
};
Box2D.Collision.Shapes.b2Shape = b2Shape;

b2Shape.prototype.Copy = function() {
   return null;
}
b2Shape.prototype.Set = function(other) {
   this.m_radius = other.m_radius;
}
b2Shape.prototype.GetType = function() {
   return this.m_type;
}
b2Shape.prototype.TestPoint = function(xf, p) {
   return false;
}
b2Shape.prototype.RayCast = function(output, input, transform) {
   return false;
}
b2Shape.prototype.ComputeAABB = function(aabb, xf) {}
b2Shape.prototype.ComputeMass = function(massData, density) {
   if (density === undefined) density = 0;
}
b2Shape.prototype.ComputeSubmergedArea = function(normal, offset, xf, c) {
   if (offset === undefined) offset = 0;
   return 0;
}
b2Shape.TestOverlap = function(shape1, transform1, shape2, transform2) {
   var input = new b2DistanceInput();
   input.proxyA = new b2DistanceProxy();
   input.proxyA.Set(shape1);
   input.proxyB = new b2DistanceProxy();
   input.proxyB.Set(shape2);
   input.transformA = transform1;
   input.transformB = transform2;
   input.useRadii = true;
   var simplexCache = new b2SimplexCache();
   simplexCache.count = 0;
   var output = new b2DistanceOutput();
   b2Distance.Distance(output, simplexCache, input);
   return output.distance < 10.0 * Number.MIN_VALUE;
}

Box2D.postDefs.push(function() {
   Box2D.Collision.Shapes.b2Shape.e_unknownShape = -1;
   Box2D.Collision.Shapes.b2Shape.e_circleShape = 0;
   Box2D.Collision.Shapes.b2Shape.e_polygonShape = 1;
   Box2D.Collision.Shapes.b2Shape.e_edgeShape = 2;
   Box2D.Collision.Shapes.b2Shape.e_shapeTypeCount = 3;
   Box2D.Collision.Shapes.b2Shape.e_hitCollide = 1;
   Box2D.Collision.Shapes.b2Shape.e_missCollide = 0;
   Box2D.Collision.Shapes.b2Shape.e_startsInsideCollide = -1;
});