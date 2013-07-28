function b2Jacobian() {
   this.linearA = new b2Vec2();
   this.linearB = new b2Vec2();

};
Box2D.Dynamics.Joints.b2Jacobian = b2Jacobian;

b2Jacobian.prototype.SetZero = function() {
   this.linearA.SetZero();
   this.angularA = 0.0;
   this.linearB.SetZero();
   this.angularB = 0.0;
}
b2Jacobian.prototype.Set = function(x1, a1, x2, a2) {
   if (a1 === undefined) a1 = 0;
   if (a2 === undefined) a2 = 0;
   this.linearA.SetV(x1);
   this.angularA = a1;
   this.linearB.SetV(x2);
   this.angularB = a2;
}
b2Jacobian.prototype.Compute = function(x1, a1, x2, a2) {
   if (a1 === undefined) a1 = 0;
   if (a2 === undefined) a2 = 0;
   return (this.linearA.x * x1.x + this.linearA.y * x1.y) + this.angularA * a1 + (this.linearB.x * x2.x + this.linearB.y * x2.y) + this.angularB * a2;
}