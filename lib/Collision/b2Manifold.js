function b2Manifold() {
   this.m_pointCount = 0;
   this.m_points = new Vector(b2Settings.b2_maxManifoldPoints);
   for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
      this.m_points[i] = new b2ManifoldPoint();
   }
   this.m_localPlaneNormal = new b2Vec2();
   this.m_localPoint = new b2Vec2();

};
Box2D.Collision.b2Manifold = b2Manifold;

b2Manifold.prototype.Reset = function() {
   for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
      ((this.m_points[i] instanceof b2ManifoldPoint ? this.m_points[i] : null)).Reset();
   }
   this.m_localPlaneNormal.SetZero();
   this.m_localPoint.SetZero();
   this.m_type = 0;
   this.m_pointCount = 0;
}
b2Manifold.prototype.Set = function(m) {
   this.m_pointCount = m.m_pointCount;
   for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
      ((this.m_points[i] instanceof b2ManifoldPoint ? this.m_points[i] : null)).Set(m.m_points[i]);
   }
   this.m_localPlaneNormal.SetV(m.m_localPlaneNormal);
   this.m_localPoint.SetV(m.m_localPoint);
   this.m_type = m.m_type;
}
b2Manifold.prototype.Copy = function() {
   var copy = new b2Manifold();
   copy.Set(this);
   return copy;
}
Box2D.postDefs.push(function() {
   Box2D.Collision.b2Manifold.e_circles = 0x0001;
   Box2D.Collision.b2Manifold.e_faceA = 0x0002;
   Box2D.Collision.b2Manifold.e_faceB = 0x0004;
});