function b2Transform(pos, r) {
   this.position = new b2Vec2;
   this.R = new b2Mat22();
   if (pos === undefined) pos = null;
   if (r === undefined) r = null;
   if (pos) {
      this.position.SetV(pos);
      this.R.SetM(r);
   }

};
Box2D.Common.Math.b2Transform = b2Transform;

b2Transform.prototype.Initialize = function(pos, r) {
   this.position.SetV(pos);
   this.R.SetM(r);
}
b2Transform.prototype.SetIdentity = function() {
   this.position.SetZero();
   this.R.SetIdentity();
}
b2Transform.prototype.Set = function(x) {
   this.position.SetV(x.position);
   this.R.SetM(x.R);
}
b2Transform.prototype.GetAngle = function() {
   return Math.atan2(this.R.col1.y, this.R.col1.x);
}