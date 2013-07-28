function b2WorldManifold() {
   this.m_normal = new b2Vec2();
   this.m_points = new Vector(b2Settings.b2_maxManifoldPoints);
   for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
      this.m_points[i] = new b2Vec2();
   }
};
Box2D.Collision.b2WorldManifold = b2WorldManifold;

function ClipVertex() {
   ClipVertex.ClipVertex.apply(this, arguments);
};
Box2D.Collision.ClipVertex = ClipVertex;

function Features() {
   Features.Features.apply(this, arguments);
};
Box2D.Collision.Features = Features;

b2WorldManifold.prototype.Initialize = function(manifold, xfA, radiusA, xfB, radiusB) {
   if (radiusA === undefined) radiusA = 0;
   if (radiusB === undefined) radiusB = 0;
   if (manifold.m_pointCount == 0) {
      return;
   }
   var i = 0;
   var tVec;
   var tMat;
   var normalX = 0;
   var normalY = 0;
   var planePointX = 0;
   var planePointY = 0;
   var clipPointX = 0;
   var clipPointY = 0;
   switch (manifold.m_type) {
   case b2Manifold.e_circles:
      {
         tMat = xfA.R;
         tVec = manifold.m_localPoint;
         var pointAX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
         var pointAY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
         tMat = xfB.R;
         tVec = manifold.m_points[0].m_localPoint;
         var pointBX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
         var pointBY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
         var dX = pointBX - pointAX;
         var dY = pointBY - pointAY;
         var d2 = dX * dX + dY * dY;
         if (d2 > Number.MIN_VALUE * Number.MIN_VALUE) {
            var d = Math.sqrt(d2);
            this.m_normal.x = dX / d;
            this.m_normal.y = dY / d;
         } else {
            this.m_normal.x = 1;
            this.m_normal.y = 0;
         }
         var cAX = pointAX + radiusA * this.m_normal.x;
         var cAY = pointAY + radiusA * this.m_normal.y;
         var cBX = pointBX - radiusB * this.m_normal.x;
         var cBY = pointBY - radiusB * this.m_normal.y;
         this.m_points[0].x = 0.5 * (cAX + cBX);
         this.m_points[0].y = 0.5 * (cAY + cBY);
      }
      break;
   case b2Manifold.e_faceA:
      {
         tMat = xfA.R;
         tVec = manifold.m_localPlaneNormal;
         normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
         normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
         tMat = xfA.R;
         tVec = manifold.m_localPoint;
         planePointX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
         planePointY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
         this.m_normal.x = normalX;
         this.m_normal.y = normalY;
         for (i = 0;
         i < manifold.m_pointCount; i++) {
            tMat = xfB.R;
            tVec = manifold.m_points[i].m_localPoint;
            clipPointX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            clipPointY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            this.m_points[i].x = clipPointX + 0.5 * (radiusA - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusB) * normalX;
            this.m_points[i].y = clipPointY + 0.5 * (radiusA - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusB) * normalY;
         }
      }
      break;
   case b2Manifold.e_faceB:
      {
         tMat = xfB.R;
         tVec = manifold.m_localPlaneNormal;
         normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
         normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
         tMat = xfB.R;
         tVec = manifold.m_localPoint;
         planePointX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
         planePointY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
         this.m_normal.x = (-normalX);
         this.m_normal.y = (-normalY);
         for (i = 0;
         i < manifold.m_pointCount; i++) {
            tMat = xfA.R;
            tVec = manifold.m_points[i].m_localPoint;
            clipPointX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            clipPointY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            this.m_points[i].x = clipPointX + 0.5 * (radiusB - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusA) * normalX;
            this.m_points[i].y = clipPointY + 0.5 * (radiusB - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusA) * normalY;
         }
      }
      break;
   }
}


ClipVertex.ClipVertex = function() {
   this.v = new b2Vec2();
   this.id = new b2ContactID();
};
ClipVertex.prototype.Set = function(other) {
   this.v.SetV(other.v);
   this.id.Set(other.id);
}


Features.Features = function() {};
Object.defineProperty(Features.prototype, 'referenceEdge', {
   enumerable: false,
   configurable: true,
   get: function() {
      return this._referenceEdge;
   }
});
Object.defineProperty(Features.prototype, 'referenceEdge', {
   enumerable: false,
   configurable: true,
   set: function(value) {
      if (value === undefined) value = 0;
      this._referenceEdge = value;
      this._m_id._key = (this._m_id._key & 0xffffff00) | (this._referenceEdge & 0x000000ff);
   }
});
Object.defineProperty(Features.prototype, 'incidentEdge', {
   enumerable: false,
   configurable: true,
   get: function() {
      return this._incidentEdge;
   }
});
Object.defineProperty(Features.prototype, 'incidentEdge', {
   enumerable: false,
   configurable: true,
   set: function(value) {
      if (value === undefined) value = 0;
      this._incidentEdge = value;
      this._m_id._key = (this._m_id._key & 0xffff00ff) | ((this._incidentEdge << 8) & 0x0000ff00);
   }
});
Object.defineProperty(Features.prototype, 'incidentVertex', {
   enumerable: false,
   configurable: true,
   get: function() {
      return this._incidentVertex;
   }
});
Object.defineProperty(Features.prototype, 'incidentVertex', {
   enumerable: false,
   configurable: true,
   set: function(value) {
      if (value === undefined) value = 0;
      this._incidentVertex = value;
      this._m_id._key = (this._m_id._key & 0xff00ffff) | ((this._incidentVertex << 16) & 0x00ff0000);
   }
});
Object.defineProperty(Features.prototype, 'flip', {
   enumerable: false,
   configurable: true,
   get: function() {
      return this._flip;
   }
});
Object.defineProperty(Features.prototype, 'flip', {
   enumerable: false,
   configurable: true,
   set: function(value) {
      if (value === undefined) value = 0;
      this._flip = value;
      this._m_id._key = (this._m_id._key & 0x00ffffff) | ((this._flip << 24) & 0xff000000);
   }
});