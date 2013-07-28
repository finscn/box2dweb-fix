function b2Point() {
   this.p = new b2Vec2();

};
Box2D.Collision.b2Point = b2Point;

b2Point.prototype.Support = function(xf, vX, vY) {
   if (vX === undefined) vX = 0;
   if (vY === undefined) vY = 0;
   return this.p;
}
b2Point.prototype.GetFirstVertex = function(xf) {
   return this.p;
}