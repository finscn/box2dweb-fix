function b2SimplexVertex() {

};
Box2D.Collision.b2SimplexVertex = b2SimplexVertex;

b2SimplexVertex.prototype.Set = function(other) {
   this.wA.SetV(other.wA);
   this.wB.SetV(other.wB);
   this.w.SetV(other.w);
   this.a = other.a;
   this.indexA = other.indexA;
   this.indexB = other.indexB;
}