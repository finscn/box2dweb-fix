
   b2AABB.b2AABB = function () {
      this.lowerBound = new b2Vec2();
      this.upperBound = new b2Vec2();
   };
   b2AABB.prototype.IsValid = function () {
      var dX = this.upperBound.x - this.lowerBound.x;
      var dY = this.upperBound.y - this.lowerBound.y;
      var valid = dX >= 0.0 && dY >= 0.0;
      valid = valid && this.lowerBound.IsValid() && this.upperBound.IsValid();
      return valid;
   }
   b2AABB.prototype.GetCenter = function () {
      return new b2Vec2((this.lowerBound.x + this.upperBound.x) / 2, (this.lowerBound.y + this.upperBound.y) / 2);
   }
   b2AABB.prototype.GetExtents = function () {
      return new b2Vec2((this.upperBound.x - this.lowerBound.x) / 2, (this.upperBound.y - this.lowerBound.y) / 2);
   }
   b2AABB.prototype.Contains = function (aabb) {
      var result = true;
      result = result && this.lowerBound.x <= aabb.lowerBound.x;
      result = result && this.lowerBound.y <= aabb.lowerBound.y;
      result = result && aabb.upperBound.x <= this.upperBound.x;
      result = result && aabb.upperBound.y <= this.upperBound.y;
      return result;
   }
   b2AABB.prototype.RayCast = function (output, input) {
      var tmin = (-Number.MAX_VALUE);
      var tmax = Number.MAX_VALUE;
      var pX = input.p1.x;
      var pY = input.p1.y;
      var dX = input.p2.x - input.p1.x;
      var dY = input.p2.y - input.p1.y;
      var absDX = Math.abs(dX);
      var absDY = Math.abs(dY);
      var normal = output.normal;
      var inv_d = 0;
      var t1 = 0;
      var t2 = 0;
      var t3 = 0;
      var s = 0; {
         if (absDX < Number.MIN_VALUE) {
            if (pX < this.lowerBound.x || this.upperBound.x < pX) return false;
         }
         else {
            inv_d = 1.0 / dX;
            t1 = (this.lowerBound.x - pX) * inv_d;
            t2 = (this.upperBound.x - pX) * inv_d;
            s = (-1.0);
            if (t1 > t2) {
               t3 = t1;
               t1 = t2;
               t2 = t3;
               s = 1.0;
            }
            if (t1 > tmin) {
               normal.x = s;
               normal.y = 0;
               tmin = t1;
            }
            tmax = Math.min(tmax, t2);
            if (tmin > tmax) return false;
         }
      } {
         if (absDY < Number.MIN_VALUE) {
            if (pY < this.lowerBound.y || this.upperBound.y < pY) return false;
         }
         else {
            inv_d = 1.0 / dY;
            t1 = (this.lowerBound.y - pY) * inv_d;
            t2 = (this.upperBound.y - pY) * inv_d;
            s = (-1.0);
            if (t1 > t2) {
               t3 = t1;
               t1 = t2;
               t2 = t3;
               s = 1.0;
            }
            if (t1 > tmin) {
               normal.y = s;
               normal.x = 0;
               tmin = t1;
            }
            tmax = Math.min(tmax, t2);
            if (tmin > tmax) return false;
         }
      }
      output.fraction = tmin;
      return true;
   }
   b2AABB.prototype.TestOverlap = function (other) {
      var d1X = other.lowerBound.x - this.upperBound.x;
      var d1Y = other.lowerBound.y - this.upperBound.y;
      var d2X = this.lowerBound.x - other.upperBound.x;
      var d2Y = this.lowerBound.y - other.upperBound.y;
      if (d1X > 0.0 || d1Y > 0.0) return false;
      if (d2X > 0.0 || d2Y > 0.0) return false;
      return true;
   }
   b2AABB.Combine = function (aabb1, aabb2) {
      var aabb = new b2AABB();
      aabb.Combine(aabb1, aabb2);
      return aabb;
   }
   b2AABB.prototype.Combine = function (aabb1, aabb2) {
      this.lowerBound.x = Math.min(aabb1.lowerBound.x, aabb2.lowerBound.x);
      this.lowerBound.y = Math.min(aabb1.lowerBound.y, aabb2.lowerBound.y);
      this.upperBound.x = Math.max(aabb1.upperBound.x, aabb2.upperBound.x);
      this.upperBound.y = Math.max(aabb1.upperBound.y, aabb2.upperBound.y);
   }

  

   b2Bound.b2Bound = function () {};
   b2Bound.prototype.IsLower = function () {
      return (this.value & 1) == 0;
   }
   b2Bound.prototype.IsUpper = function () {
      return (this.value & 1) == 1;
   }
   b2Bound.prototype.Swap = function (b) {
      var tempValue = this.value;
      var tempProxy = this.proxy;
      var tempStabbingCount = this.stabbingCount;
      this.value = b.value;
      this.proxy = b.proxy;
      this.stabbingCount = b.stabbingCount;
      b.value = tempValue;
      b.proxy = tempProxy;
      b.stabbingCount = tempStabbingCount;
   }



   b2BoundValues.b2BoundValues = function () {};
   b2BoundValues.prototype.b2BoundValues = function () {
      this.lowerValues = new Vector_a2j_Number();
      this.lowerValues[0] = 0.0;
      this.lowerValues[1] = 0.0;
      this.upperValues = new Vector_a2j_Number();
      this.upperValues[0] = 0.0;
      this.upperValues[1] = 0.0;
   }




   b2Collision.b2Collision = function () {};
   b2Collision.ClipSegmentToLine = function (vOut, vIn, normal, offset) {
      if (offset === undefined) offset = 0;
      var cv;
      var numOut = 0;
      cv = vIn[0];
      var vIn0 = cv.v;
      cv = vIn[1];
      var vIn1 = cv.v;
      var distance0 = normal.x * vIn0.x + normal.y * vIn0.y - offset;
      var distance1 = normal.x * vIn1.x + normal.y * vIn1.y - offset;
      if (distance0 <= 0.0) vOut[numOut++].Set(vIn[0]);
      if (distance1 <= 0.0) vOut[numOut++].Set(vIn[1]);
      if (distance0 * distance1 < 0.0) {
         var interp = distance0 / (distance0 - distance1);
         cv = vOut[numOut];
         var tVec = cv.v;
         tVec.x = vIn0.x + interp * (vIn1.x - vIn0.x);
         tVec.y = vIn0.y + interp * (vIn1.y - vIn0.y);
         cv = vOut[numOut];
         var cv2;
         if (distance0 > 0.0) {
            cv2 = vIn[0];
            cv.id = cv2.id;
         }
         else {
            cv2 = vIn[1];
            cv.id = cv2.id;
         }++numOut;
      }
      return numOut;
   }
   b2Collision.EdgeSeparation = function (poly1, xf1, edge1, poly2, xf2) {
      if (edge1 === undefined) edge1 = 0;
      var count1 = parseInt(poly1.m_vertexCount);
      var vertices1 = poly1.m_vertices;
      var normals1 = poly1.m_normals;
      var count2 = parseInt(poly2.m_vertexCount);
      var vertices2 = poly2.m_vertices;
      var tMat;
      var tVec;
      tMat = xf1.R;
      tVec = normals1[edge1];
      var normal1WorldX = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var normal1WorldY = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tMat = xf2.R;
      var normal1X = (tMat.col1.x * normal1WorldX + tMat.col1.y * normal1WorldY);
      var normal1Y = (tMat.col2.x * normal1WorldX + tMat.col2.y * normal1WorldY);
      var index = 0;
      var minDot = Number.MAX_VALUE;
      for (var i = 0; i < count2; ++i) {
         tVec = vertices2[i];
         var dot = tVec.x * normal1X + tVec.y * normal1Y;
         if (dot < minDot) {
            minDot = dot;
            index = i;
         }
      }
      tVec = vertices1[edge1];
      tMat = xf1.R;
      var v1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var v1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tVec = vertices2[index];
      tMat = xf2.R;
      var v2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var v2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      v2X -= v1X;
      v2Y -= v1Y;
      var separation = v2X * normal1WorldX + v2Y * normal1WorldY;
      return separation;
   }
   b2Collision.FindMaxSeparation = function (edgeIndex, poly1, xf1, poly2, xf2) {
      var count1 = parseInt(poly1.m_vertexCount);
      var normals1 = poly1.m_normals;
      var tVec;
      var tMat;
      tMat = xf2.R;
      tVec = poly2.m_centroid;
      var dX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var dY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tMat = xf1.R;
      tVec = poly1.m_centroid;
      dX -= xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      dY -= xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      var dLocal1X = (dX * xf1.R.col1.x + dY * xf1.R.col1.y);
      var dLocal1Y = (dX * xf1.R.col2.x + dY * xf1.R.col2.y);
      var edge = 0;
      var maxDot = (-Number.MAX_VALUE);
      for (var i = 0; i < count1; ++i) {
         tVec = normals1[i];
         var dot = (tVec.x * dLocal1X + tVec.y * dLocal1Y);
         if (dot > maxDot) {
            maxDot = dot;
            edge = i;
         }
      }
      var s = b2Collision.EdgeSeparation(poly1, xf1, edge, poly2, xf2);
      var prevEdge = parseInt(edge - 1 >= 0 ? edge - 1 : count1 - 1);
      var sPrev = b2Collision.EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
      var nextEdge = parseInt(edge + 1 < count1 ? edge + 1 : 0);
      var sNext = b2Collision.EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);
      var bestEdge = 0;
      var bestSeparation = 0;
      var increment = 0;
      if (sPrev > s && sPrev > sNext) {
         increment = (-1);
         bestEdge = prevEdge;
         bestSeparation = sPrev;
      }
      else if (sNext > s) {
         increment = 1;
         bestEdge = nextEdge;
         bestSeparation = sNext;
      }
      else {
         edgeIndex[0] = edge;
         return s;
      }
      while (true) {
         if (increment == (-1)) edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
         else edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;s = b2Collision.EdgeSeparation(poly1, xf1, edge, poly2, xf2);
         if (s > bestSeparation) {
            bestEdge = edge;
            bestSeparation = s;
         }
         else {
            break;
         }
      }
      edgeIndex[0] = bestEdge;
      return bestSeparation;
   }
   b2Collision.FindIncidentEdge = function (c, poly1, xf1, edge1, poly2, xf2) {
      if (edge1 === undefined) edge1 = 0;
      var count1 = parseInt(poly1.m_vertexCount);
      var normals1 = poly1.m_normals;
      var count2 = parseInt(poly2.m_vertexCount);
      var vertices2 = poly2.m_vertices;
      var normals2 = poly2.m_normals;
      var tMat;
      var tVec;
      tMat = xf1.R;
      tVec = normals1[edge1];
      var normal1X = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var normal1Y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tMat = xf2.R;
      var tX = (tMat.col1.x * normal1X + tMat.col1.y * normal1Y);
      normal1Y = (tMat.col2.x * normal1X + tMat.col2.y * normal1Y);
      normal1X = tX;
      var index = 0;
      var minDot = Number.MAX_VALUE;
      for (var i = 0; i < count2; ++i) {
         tVec = normals2[i];
         var dot = (normal1X * tVec.x + normal1Y * tVec.y);
         if (dot < minDot) {
            minDot = dot;
            index = i;
         }
      }
      var tClip;
      var i1 = parseInt(index);
      var i2 = parseInt(i1 + 1 < count2 ? i1 + 1 : 0);
      tClip = c[0];
      tVec = vertices2[i1];
      tMat = xf2.R;
      tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tClip.id.features.referenceEdge = edge1;
      tClip.id.features.incidentEdge = i1;
      tClip.id.features.incidentVertex = 0;
      tClip = c[1];
      tVec = vertices2[i2];
      tMat = xf2.R;
      tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tClip.id.features.referenceEdge = edge1;
      tClip.id.features.incidentEdge = i2;
      tClip.id.features.incidentVertex = 1;
   }
   b2Collision.MakeClipPointVector = function () {
      var r = new Vector(2);
      r[0] = new ClipVertex();
      r[1] = new ClipVertex();
      return r;
   }
   b2Collision.CollidePolygons = function (manifold, polyA, xfA, polyB, xfB) {
      var cv;
      manifold.m_pointCount = 0;
      var totalRadius = polyA.m_radius + polyB.m_radius;
      var edgeA = 0;
      b2Collision.s_edgeAO[0] = edgeA;
      var separationA = b2Collision.FindMaxSeparation(b2Collision.s_edgeAO, polyA, xfA, polyB, xfB);
      edgeA = b2Collision.s_edgeAO[0];
      if (separationA > totalRadius) return;
      var edgeB = 0;
      b2Collision.s_edgeBO[0] = edgeB;
      var separationB = b2Collision.FindMaxSeparation(b2Collision.s_edgeBO, polyB, xfB, polyA, xfA);
      edgeB = b2Collision.s_edgeBO[0];
      if (separationB > totalRadius) return;
      var poly1;
      var poly2;
      var xf1;
      var xf2;
      var edge1 = 0;
      var flip = 0;
      var k_relativeTol = 0.98;
      var k_absoluteTol = 0.001;
      var tMat;
      if (separationB > k_relativeTol * separationA + k_absoluteTol) {
         poly1 = polyB;
         poly2 = polyA;
         xf1 = xfB;
         xf2 = xfA;
         edge1 = edgeB;
         manifold.m_type = b2Manifold.e_faceB;
         flip = 1;
      }
      else {
         poly1 = polyA;
         poly2 = polyB;
         xf1 = xfA;
         xf2 = xfB;
         edge1 = edgeA;
         manifold.m_type = b2Manifold.e_faceA;
         flip = 0;
      }
      var incidentEdge = b2Collision.s_incidentEdge;
      b2Collision.FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);
      var count1 = parseInt(poly1.m_vertexCount);
      var vertices1 = poly1.m_vertices;
      var local_v11 = vertices1[edge1];
      var local_v12;
      if (edge1 + 1 < count1) {
         local_v12 = vertices1[parseInt(edge1 + 1)];
      }
      else {
         local_v12 = vertices1[0];
      }
      var localTangent = b2Collision.s_localTangent;
      localTangent.Set(local_v12.x - local_v11.x, local_v12.y - local_v11.y);
      localTangent.Normalize();
      var localNormal = b2Collision.s_localNormal;
      localNormal.x = localTangent.y;
      localNormal.y = (-localTangent.x);
      var planePoint = b2Collision.s_planePoint;
      planePoint.Set(0.5 * (local_v11.x + local_v12.x), 0.5 * (local_v11.y + local_v12.y));
      var tangent = b2Collision.s_tangent;
      tMat = xf1.R;
      tangent.x = (tMat.col1.x * localTangent.x + tMat.col2.x * localTangent.y);
      tangent.y = (tMat.col1.y * localTangent.x + tMat.col2.y * localTangent.y);
      var tangent2 = b2Collision.s_tangent2;
      tangent2.x = (-tangent.x);
      tangent2.y = (-tangent.y);
      var normal = b2Collision.s_normal;
      normal.x = tangent.y;
      normal.y = (-tangent.x);
      var v11 = b2Collision.s_v11;
      var v12 = b2Collision.s_v12;
      v11.x = xf1.position.x + (tMat.col1.x * local_v11.x + tMat.col2.x * local_v11.y);
      v11.y = xf1.position.y + (tMat.col1.y * local_v11.x + tMat.col2.y * local_v11.y);
      v12.x = xf1.position.x + (tMat.col1.x * local_v12.x + tMat.col2.x * local_v12.y);
      v12.y = xf1.position.y + (tMat.col1.y * local_v12.x + tMat.col2.y * local_v12.y);
      var frontOffset = normal.x * v11.x + normal.y * v11.y;
      var sideOffset1 = (-tangent.x * v11.x) - tangent.y * v11.y + totalRadius;
      var sideOffset2 = tangent.x * v12.x + tangent.y * v12.y + totalRadius;
      var clipPoints1 = b2Collision.s_clipPoints1;
      var clipPoints2 = b2Collision.s_clipPoints2;
      var np = 0;
      np = b2Collision.ClipSegmentToLine(clipPoints1, incidentEdge, tangent2, sideOffset1);
      if (np < 2) return;
      np = b2Collision.ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2);
      if (np < 2) return;
      manifold.m_localPlaneNormal.SetV(localNormal);
      manifold.m_localPoint.SetV(planePoint);
      var pointCount = 0;
      for (var i = 0; i < b2Settings.b2_maxManifoldPoints; ++i) {
         cv = clipPoints2[i];
         var separation = normal.x * cv.v.x + normal.y * cv.v.y - frontOffset;
         if (separation <= totalRadius) {
            var cp = manifold.m_points[pointCount];
            tMat = xf2.R;
            var tX = cv.v.x - xf2.position.x;
            var tY = cv.v.y - xf2.position.y;
            cp.m_localPoint.x = (tX * tMat.col1.x + tY * tMat.col1.y);
            cp.m_localPoint.y = (tX * tMat.col2.x + tY * tMat.col2.y);
            cp.m_id.Set(cv.id);
            cp.m_id.features.flip = flip;
            ++pointCount;
         }
      }
      manifold.m_pointCount = pointCount;
   }
   b2Collision.CollideCircles = function (manifold, circle1, xf1, circle2, xf2) {
      manifold.m_pointCount = 0;
      var tMat;
      var tVec;
      tMat = xf1.R;
      tVec = circle1.m_p;
      var p1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var p1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tMat = xf2.R;
      tVec = circle2.m_p;
      var p2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var p2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      var dX = p2X - p1X;
      var dY = p2Y - p1Y;
      var distSqr = dX * dX + dY * dY;
      var radius = circle1.m_radius + circle2.m_radius;
      if (distSqr > radius * radius) {
         return;
      }
      manifold.m_type = b2Manifold.e_circles;
      manifold.m_localPoint.SetV(circle1.m_p);
      manifold.m_localPlaneNormal.SetZero();
      manifold.m_pointCount = 1;
      manifold.m_points[0].m_localPoint.SetV(circle2.m_p);
      manifold.m_points[0].m_id.key = 0;
   }
   b2Collision.CollidePolygonAndCircle = function (manifold, polygon, xf1, circle, xf2) {
      manifold.m_pointCount = 0;
      var tPoint;
      var dX = 0;
      var dY = 0;
      var positionX = 0;
      var positionY = 0;
      var tVec;
      var tMat;
      tMat = xf2.R;
      tVec = circle.m_p;
      var cX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var cY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      dX = cX - xf1.position.x;
      dY = cY - xf1.position.y;
      tMat = xf1.R;
      var cLocalX = (dX * tMat.col1.x + dY * tMat.col1.y);
      var cLocalY = (dX * tMat.col2.x + dY * tMat.col2.y);
      var dist = 0;
      var normalIndex = 0;
      var separation = (-Number.MAX_VALUE);
      var radius = polygon.m_radius + circle.m_radius;
      var vertexCount = parseInt(polygon.m_vertexCount);
      var vertices = polygon.m_vertices;
      var normals = polygon.m_normals;
      for (var i = 0; i < vertexCount; ++i) {
         tVec = vertices[i];
         dX = cLocalX - tVec.x;
         dY = cLocalY - tVec.y;
         tVec = normals[i];
         var s = tVec.x * dX + tVec.y * dY;
         if (s > radius) {
            return;
         }
         if (s > separation) {
            separation = s;
            normalIndex = i;
         }
      }
      var vertIndex1 = parseInt(normalIndex);
      var vertIndex2 = parseInt(vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0);
      var v1 = vertices[vertIndex1];
      var v2 = vertices[vertIndex2];
      if (separation < Number.MIN_VALUE) {
         manifold.m_pointCount = 1;
         manifold.m_type = b2Manifold.e_faceA;
         manifold.m_localPlaneNormal.SetV(normals[normalIndex]);
         manifold.m_localPoint.x = 0.5 * (v1.x + v2.x);
         manifold.m_localPoint.y = 0.5 * (v1.y + v2.y);
         manifold.m_points[0].m_localPoint.SetV(circle.m_p);
         manifold.m_points[0].m_id.key = 0;
         return;
      }
      var u1 = (cLocalX - v1.x) * (v2.x - v1.x) + (cLocalY - v1.y) * (v2.y - v1.y);
      var u2 = (cLocalX - v2.x) * (v1.x - v2.x) + (cLocalY - v2.y) * (v1.y - v2.y);
      if (u1 <= 0.0) {
         if ((cLocalX - v1.x) * (cLocalX - v1.x) + (cLocalY - v1.y) * (cLocalY - v1.y) > radius * radius) return;
         manifold.m_pointCount = 1;
         manifold.m_type = b2Manifold.e_faceA;
         manifold.m_localPlaneNormal.x = cLocalX - v1.x;
         manifold.m_localPlaneNormal.y = cLocalY - v1.y;
         manifold.m_localPlaneNormal.Normalize();
         manifold.m_localPoint.SetV(v1);
         manifold.m_points[0].m_localPoint.SetV(circle.m_p);
         manifold.m_points[0].m_id.key = 0;
      }
      else if (u2 <= 0) {
         if ((cLocalX - v2.x) * (cLocalX - v2.x) + (cLocalY - v2.y) * (cLocalY - v2.y) > radius * radius) return;
         manifold.m_pointCount = 1;
         manifold.m_type = b2Manifold.e_faceA;
         manifold.m_localPlaneNormal.x = cLocalX - v2.x;
         manifold.m_localPlaneNormal.y = cLocalY - v2.y;
         manifold.m_localPlaneNormal.Normalize();
         manifold.m_localPoint.SetV(v2);
         manifold.m_points[0].m_localPoint.SetV(circle.m_p);
         manifold.m_points[0].m_id.key = 0;
      }
      else {
         var faceCenterX = 0.5 * (v1.x + v2.x);
         var faceCenterY = 0.5 * (v1.y + v2.y);
         separation = (cLocalX - faceCenterX) * normals[vertIndex1].x + (cLocalY - faceCenterY) * normals[vertIndex1].y;
         if (separation > radius) return;
         manifold.m_pointCount = 1;
         manifold.m_type = b2Manifold.e_faceA;
         manifold.m_localPlaneNormal.x = normals[vertIndex1].x;
         manifold.m_localPlaneNormal.y = normals[vertIndex1].y;
         manifold.m_localPlaneNormal.Normalize();
         manifold.m_localPoint.Set(faceCenterX, faceCenterY);
         manifold.m_points[0].m_localPoint.SetV(circle.m_p);
         manifold.m_points[0].m_id.key = 0;
      }
   }
   b2Collision.TestOverlap = function (a, b) {
      var t1 = b.lowerBound;
      var t2 = a.upperBound;
      var d1X = t1.x - t2.x;
      var d1Y = t1.y - t2.y;
      t1 = a.lowerBound;
      t2 = b.upperBound;
      var d2X = t1.x - t2.x;
      var d2Y = t1.y - t2.y;
      if (d1X > 0.0 || d1Y > 0.0) return false;
      if (d2X > 0.0 || d2Y > 0.0) return false;
      return true;
   }
   Box2D.postDefs.push(function () {
      Box2D.Collision.b2Collision.s_incidentEdge = b2Collision.MakeClipPointVector();
      Box2D.Collision.b2Collision.s_clipPoints1 = b2Collision.MakeClipPointVector();
      Box2D.Collision.b2Collision.s_clipPoints2 = b2Collision.MakeClipPointVector();
      Box2D.Collision.b2Collision.s_edgeAO = new Vector_a2j_Number(1);
      Box2D.Collision.b2Collision.s_edgeBO = new Vector_a2j_Number(1);
      Box2D.Collision.b2Collision.s_localTangent = new b2Vec2();
      Box2D.Collision.b2Collision.s_localNormal = new b2Vec2();
      Box2D.Collision.b2Collision.s_planePoint = new b2Vec2();
      Box2D.Collision.b2Collision.s_normal = new b2Vec2();
      Box2D.Collision.b2Collision.s_tangent = new b2Vec2();
      Box2D.Collision.b2Collision.s_tangent2 = new b2Vec2();
      Box2D.Collision.b2Collision.s_v11 = new b2Vec2();
      Box2D.Collision.b2Collision.s_v12 = new b2Vec2();
      Box2D.Collision.b2Collision.b2CollidePolyTempVec = new b2Vec2();
      Box2D.Collision.b2Collision.b2_nullFeature = 0x000000ff;
   });



   b2ContactID.b2ContactID = function () {
      this.features = new Features();
   };
   b2ContactID.prototype.b2ContactID = function () {
      this.features._m_id = this;
   }
   b2ContactID.prototype.Set = function (id) {
      this.key = id._key;
   }
   b2ContactID.prototype.Copy = function () {
      var id = new b2ContactID();
      id.key = this.key;
      return id;
   }
   Object.defineProperty(b2ContactID.prototype, 'key', {
      enumerable: false,
      configurable: true,
      get: function () {
         return this._key;
      }
   });
   Object.defineProperty(b2ContactID.prototype, 'key', {
      enumerable: false,
      configurable: true,
      set: function (value) {
         if (value === undefined) value = 0;
         this._key = value;
         this.features._referenceEdge = this._key & 0x000000ff;
         this.features._incidentEdge = ((this._key & 0x0000ff00) >> 8) & 0x000000ff;
         this.features._incidentVertex = ((this._key & 0x00ff0000) >> 16) & 0x000000ff;
         this.features._flip = ((this._key & 0xff000000) >> 24) & 0x000000ff;
      }
   });



   b2ContactPoint.b2ContactPoint = function () {
      this.position = new b2Vec2();
      this.velocity = new b2Vec2();
      this.normal = new b2Vec2();
      this.id = new b2ContactID();
   };




   b2Distance.b2Distance = function () {};
   b2Distance.Distance = function (output, cache, input) {
      ++b2Distance.b2_gjkCalls;
      var proxyA = input.proxyA;
      var proxyB = input.proxyB;
      var transformA = input.transformA;
      var transformB = input.transformB;
      var simplex = b2Distance.s_simplex;
      simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);
      var vertices = simplex.m_vertices;
      var k_maxIters = 20;
      var saveA = b2Distance.s_saveA;
      var saveB = b2Distance.s_saveB;
      var saveCount = 0;
      var closestPoint = simplex.GetClosestPoint();
      var distanceSqr1 = closestPoint.LengthSquared();
      var distanceSqr2 = distanceSqr1;
      var i = 0;
      var p;
      var iter = 0;
      while (iter < k_maxIters) {
         saveCount = simplex.m_count;
         for (i = 0;
         i < saveCount; i++) {
            saveA[i] = vertices[i].indexA;
            saveB[i] = vertices[i].indexB;
         }
         switch (simplex.m_count) {
         case 1:
            break;
         case 2:
            simplex.Solve2();
            break;
         case 3:
            simplex.Solve3();
            break;
         default:
            b2Settings.b2Assert(false);
         }
         if (simplex.m_count == 3) {
            break;
         }
         p = simplex.GetClosestPoint();
         distanceSqr2 = p.LengthSquared();
         if (distanceSqr2 > distanceSqr1) {}
         distanceSqr1 = distanceSqr2;
         var d = simplex.GetSearchDirection();
         if (d.LengthSquared() < Number.MIN_VALUE * Number.MIN_VALUE) {
            break;
         }
         var vertex = vertices[simplex.m_count];
         vertex.indexA = proxyA.GetSupport(b2Math.MulTMV(transformA.R, d.GetNegative()));
         vertex.wA = b2Math.MulX(transformA, proxyA.GetVertex(vertex.indexA));
         vertex.indexB = proxyB.GetSupport(b2Math.MulTMV(transformB.R, d));
         vertex.wB = b2Math.MulX(transformB, proxyB.GetVertex(vertex.indexB));
         vertex.w = b2Math.SubtractVV(vertex.wB, vertex.wA);
         ++iter;
         ++b2Distance.b2_gjkIters;
         var duplicate = false;
         for (i = 0;
         i < saveCount; i++) {
            if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i]) {
               duplicate = true;
               break;
            }
         }
         if (duplicate) {
            break;
         }++simplex.m_count;
      }
      b2Distance.b2_gjkMaxIters = b2Math.Max(b2Distance.b2_gjkMaxIters, iter);
      simplex.GetWitnessPoints(output.pointA, output.pointB);
      output.distance = b2Math.SubtractVV(output.pointA, output.pointB).Length();
      output.iterations = iter;
      simplex.WriteCache(cache);
      if (input.useRadii) {
         var rA = proxyA.m_radius;
         var rB = proxyB.m_radius;
         if (output.distance > rA + rB && output.distance > Number.MIN_VALUE) {
            output.distance -= rA + rB;
            var normal = b2Math.SubtractVV(output.pointB, output.pointA);
            normal.Normalize();
            output.pointA.x += rA * normal.x;
            output.pointA.y += rA * normal.y;
            output.pointB.x -= rB * normal.x;
            output.pointB.y -= rB * normal.y;
         }
         else {
            p = new b2Vec2();
            p.x = .5 * (output.pointA.x + output.pointB.x);
            p.y = .5 * (output.pointA.y + output.pointB.y);
            output.pointA.x = output.pointB.x = p.x;
            output.pointA.y = output.pointB.y = p.y;
            output.distance = 0.0;
         }
      }
   }
   Box2D.postDefs.push(function () {
      Box2D.Collision.b2Distance.s_simplex = new b2Simplex();
      Box2D.Collision.b2Distance.s_saveA = new Vector_a2j_Number(3);
      Box2D.Collision.b2Distance.s_saveB = new Vector_a2j_Number(3);
   });




   b2DistanceInput.b2DistanceInput = function () {};



   b2DistanceOutput.b2DistanceOutput = function () {
      this.pointA = new b2Vec2();
      this.pointB = new b2Vec2();
   };




   b2DistanceProxy.b2DistanceProxy = function () {};
   b2DistanceProxy.prototype.Set = function (shape) {
      switch (shape.GetType()) {
      case b2Shape.e_circleShape:
         {
            var circle = (shape instanceof b2CircleShape ? shape : null);
            this.m_vertices = new Vector(1, true);
            this.m_vertices[0] = circle.m_p;
            this.m_count = 1;
            this.m_radius = circle.m_radius;
         }
         break;
      case b2Shape.e_polygonShape:
         {
            var polygon = (shape instanceof b2PolygonShape ? shape : null);
            this.m_vertices = polygon.m_vertices;
            this.m_count = polygon.m_vertexCount;
            this.m_radius = polygon.m_radius;
         }
         break;
      default:
         b2Settings.b2Assert(false);
      }
   }
   b2DistanceProxy.prototype.GetSupport = function (d) {
      var bestIndex = 0;
      var bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
      for (var i = 1; i < this.m_count; ++i) {
         var value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
         if (value > bestValue) {
            bestIndex = i;
            bestValue = value;
         }
      }
      return bestIndex;
   }
   b2DistanceProxy.prototype.GetSupportVertex = function (d) {
      var bestIndex = 0;
      var bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
      for (var i = 1; i < this.m_count; ++i) {
         var value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
         if (value > bestValue) {
            bestIndex = i;
            bestValue = value;
         }
      }
      return this.m_vertices[bestIndex];
   }
   b2DistanceProxy.prototype.GetVertexCount = function () {
      return this.m_count;
   }
   b2DistanceProxy.prototype.GetVertex = function (index) {
      if (index === undefined) index = 0;
      b2Settings.b2Assert(0 <= index && index < this.m_count);
      return this.m_vertices[index];
   }




   b2DynamicTree.b2DynamicTree = function () {};
   b2DynamicTree.prototype.b2DynamicTree = function () {
      this.m_root = null;
      this.m_freeList = null;
      this.m_path = 0;
      this.m_insertionCount = 0;
   }
   b2DynamicTree.prototype.CreateProxy = function (aabb, userData) {
      var node = this.AllocateNode();
      var extendX = b2Settings.b2_aabbExtension;
      var extendY = b2Settings.b2_aabbExtension;
      node.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
      node.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
      node.aabb.upperBound.x = aabb.upperBound.x + extendX;
      node.aabb.upperBound.y = aabb.upperBound.y + extendY;
      node.userData = userData;
      this.InsertLeaf(node);
      return node;
   }
   b2DynamicTree.prototype.DestroyProxy = function (proxy) {
      this.RemoveLeaf(proxy);
      this.FreeNode(proxy);
   }
   b2DynamicTree.prototype.MoveProxy = function (proxy, aabb, displacement) {
      b2Settings.b2Assert(proxy.IsLeaf());
      if (proxy.aabb.Contains(aabb)) {
         return false;
      }
      this.RemoveLeaf(proxy);
      var extendX = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * (displacement.x > 0 ? displacement.x : (-displacement.x));
      var extendY = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * (displacement.y > 0 ? displacement.y : (-displacement.y));
      proxy.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
      proxy.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
      proxy.aabb.upperBound.x = aabb.upperBound.x + extendX;
      proxy.aabb.upperBound.y = aabb.upperBound.y + extendY;
      this.InsertLeaf(proxy);
      return true;
   }
   b2DynamicTree.prototype.Rebalance = function (iterations) {
      if (iterations === undefined) iterations = 0;
      if (this.m_root == null) return;
      for (var i = 0; i < iterations; i++) {
         var node = this.m_root;
         var bit = 0;
         while (node.IsLeaf() == false) {
            node = (this.m_path >> bit) & 1 ? node.child2 : node.child1;
            bit = (bit + 1) & 31;
         }++this.m_path;
         this.RemoveLeaf(node);
         this.InsertLeaf(node);
      }
   }
   b2DynamicTree.prototype.GetFatAABB = function (proxy) {
      return proxy.aabb;
   }
   b2DynamicTree.prototype.GetUserData = function (proxy) {
      return proxy.userData;
   }
   b2DynamicTree.prototype.Query = function (callback, aabb) {
      if (this.m_root == null) return;
      var stack = new Vector();
      var count = 0;
      stack[count++] = this.m_root;
      while (count > 0) {
         var node = stack[--count];
         if (node.aabb.TestOverlap(aabb)) {
            if (node.IsLeaf()) {
               var proceed = callback(node);
               if (!proceed) return;
            }
            else {
               stack[count++] = node.child1;
               stack[count++] = node.child2;
            }
         }
      }
   }
   b2DynamicTree.prototype.RayCast = function (callback, input) {
      if (this.m_root == null) return;
      var p1 = input.p1;
      var p2 = input.p2;
      var r = b2Math.SubtractVV(p1, p2);
      r.Normalize();
      var v = b2Math.CrossFV(1.0, r);
      var abs_v = b2Math.AbsV(v);
      var maxFraction = input.maxFraction;
      var segmentAABB = new b2AABB();
      var tX = 0;
      var tY = 0; {
         tX = p1.x + maxFraction * (p2.x - p1.x);
         tY = p1.y + maxFraction * (p2.y - p1.y);
         segmentAABB.lowerBound.x = Math.min(p1.x, tX);
         segmentAABB.lowerBound.y = Math.min(p1.y, tY);
         segmentAABB.upperBound.x = Math.max(p1.x, tX);
         segmentAABB.upperBound.y = Math.max(p1.y, tY);
      }
      var stack = new Vector();
      var count = 0;
      stack[count++] = this.m_root;
      while (count > 0) {
         var node = stack[--count];
         if (node.aabb.TestOverlap(segmentAABB) == false) {
            continue;
         }
         var c = node.aabb.GetCenter();
         var h = node.aabb.GetExtents();
         var separation = Math.abs(v.x * (p1.x - c.x) + v.y * (p1.y - c.y)) - abs_v.x * h.x - abs_v.y * h.y;
         if (separation > 0.0) continue;
         if (node.IsLeaf()) {
            var subInput = new b2RayCastInput();
            subInput.p1 = input.p1;
            subInput.p2 = input.p2;
            subInput.maxFraction = input.maxFraction;
            maxFraction = callback(subInput, node);
            if (maxFraction == 0.0) return;
            if (maxFraction > 0.0) {
               tX = p1.x + maxFraction * (p2.x - p1.x);
               tY = p1.y + maxFraction * (p2.y - p1.y);
               segmentAABB.lowerBound.x = Math.min(p1.x, tX);
               segmentAABB.lowerBound.y = Math.min(p1.y, tY);
               segmentAABB.upperBound.x = Math.max(p1.x, tX);
               segmentAABB.upperBound.y = Math.max(p1.y, tY);
            }
         }
         else {
            stack[count++] = node.child1;
            stack[count++] = node.child2;
         }
      }
   }
   b2DynamicTree.prototype.AllocateNode = function () {
      if (this.m_freeList) {
         var node = this.m_freeList;
         this.m_freeList = node.parent;
         node.parent = null;
         node.child1 = null;
         node.child2 = null;
         return node;
      }
      return new b2DynamicTreeNode();
   }
   b2DynamicTree.prototype.FreeNode = function (node) {
      node.parent = this.m_freeList;
      this.m_freeList = node;
   }
   b2DynamicTree.prototype.InsertLeaf = function (leaf) {
      ++this.m_insertionCount;
      if (this.m_root == null) {
         this.m_root = leaf;
         this.m_root.parent = null;
         return;
      }
      var center = leaf.aabb.GetCenter();
      var sibling = this.m_root;
      if (sibling.IsLeaf() == false) {
         do {
            var child1 = sibling.child1;
            var child2 = sibling.child2;
            var norm1 = Math.abs((child1.aabb.lowerBound.x + child1.aabb.upperBound.x) / 2 - center.x) + Math.abs((child1.aabb.lowerBound.y + child1.aabb.upperBound.y) / 2 - center.y);
            var norm2 = Math.abs((child2.aabb.lowerBound.x + child2.aabb.upperBound.x) / 2 - center.x) + Math.abs((child2.aabb.lowerBound.y + child2.aabb.upperBound.y) / 2 - center.y);
            if (norm1 < norm2) {
               sibling = child1;
            }
            else {
               sibling = child2;
            }
         }
         while (sibling.IsLeaf() == false)
      }
      var node1 = sibling.parent;
      var node2 = this.AllocateNode();
      node2.parent = node1;
      node2.userData = null;
      node2.aabb.Combine(leaf.aabb, sibling.aabb);
      if (node1) {
         if (sibling.parent.child1 == sibling) {
            node1.child1 = node2;
         }
         else {
            node1.child2 = node2;
         }
         node2.child1 = sibling;
         node2.child2 = leaf;
         sibling.parent = node2;
         leaf.parent = node2;
         do {
            if (node1.aabb.Contains(node2.aabb)) break;
            node1.aabb.Combine(node1.child1.aabb, node1.child2.aabb);
            node2 = node1;
            node1 = node1.parent;
         }
         while (node1)
      }
      else {
         node2.child1 = sibling;
         node2.child2 = leaf;
         sibling.parent = node2;
         leaf.parent = node2;
         this.m_root = node2;
      }
   }
   b2DynamicTree.prototype.RemoveLeaf = function (leaf) {
      if (leaf == this.m_root) {
         this.m_root = null;
         return;
      }
      var node2 = leaf.parent;
      var node1 = node2.parent;
      var sibling;
      if (node2.child1 == leaf) {
         sibling = node2.child2;
      }
      else {
         sibling = node2.child1;
      }
      if (node1) {
         if (node1.child1 == node2) {
            node1.child1 = sibling;
         }
         else {
            node1.child2 = sibling;
         }
         sibling.parent = node1;
         this.FreeNode(node2);
         while (node1) {
            var oldAABB = node1.aabb;
            node1.aabb = b2AABB.Combine(node1.child1.aabb, node1.child2.aabb);
            if (oldAABB.Contains(node1.aabb)) break;
            node1 = node1.parent;
         }
      }
      else {
         this.m_root = sibling;
         sibling.parent = null;
         this.FreeNode(node2);
      }
   }




   b2DynamicTreeBroadPhase.b2DynamicTreeBroadPhase = function () {
      this.m_tree = new b2DynamicTree();
      this.m_moveBuffer = new Vector();
      this.m_pairBuffer = new Vector();
      this.m_pairCount = 0;
   };
   b2DynamicTreeBroadPhase.prototype.CreateProxy = function (aabb, userData) {
      var proxy = this.m_tree.CreateProxy(aabb, userData);
      ++this.m_proxyCount;
      this.BufferMove(proxy);
      return proxy;
   }
   b2DynamicTreeBroadPhase.prototype.DestroyProxy = function (proxy) {
      this.UnBufferMove(proxy);
      --this.m_proxyCount;
      this.m_tree.DestroyProxy(proxy);
   }
   b2DynamicTreeBroadPhase.prototype.MoveProxy = function (proxy, aabb, displacement) {
      var buffer = this.m_tree.MoveProxy(proxy, aabb, displacement);
      if (buffer) {
         this.BufferMove(proxy);
      }
   }
   b2DynamicTreeBroadPhase.prototype.TestOverlap = function (proxyA, proxyB) {
      var aabbA = this.m_tree.GetFatAABB(proxyA);
      var aabbB = this.m_tree.GetFatAABB(proxyB);
      return aabbA.TestOverlap(aabbB);
   }
   b2DynamicTreeBroadPhase.prototype.GetUserData = function (proxy) {
      return this.m_tree.GetUserData(proxy);
   }
   b2DynamicTreeBroadPhase.prototype.GetFatAABB = function (proxy) {
      return this.m_tree.GetFatAABB(proxy);
   }
   b2DynamicTreeBroadPhase.prototype.GetProxyCount = function () {
      return this.m_proxyCount;
   }
   b2DynamicTreeBroadPhase.prototype.UpdatePairs = function (callback) {
      var __this = this;
      __this.m_pairCount = 0;
      var i = 0,
         queryProxy;
      for (i = 0;
      i < __this.m_moveBuffer.length; ++i) {
         queryProxy = __this.m_moveBuffer[i];

         function QueryCallback(proxy) {
            if (proxy == queryProxy) return true;
            if (__this.m_pairCount == __this.m_pairBuffer.length) {
               __this.m_pairBuffer[__this.m_pairCount] = new b2DynamicTreePair();
            }
            var pair = __this.m_pairBuffer[__this.m_pairCount];
            pair.proxyA = proxy < queryProxy ? proxy : queryProxy;
            pair.proxyB = proxy >= queryProxy ? proxy : queryProxy;++__this.m_pairCount;
            return true;
         };
         var fatAABB = __this.m_tree.GetFatAABB(queryProxy);
         __this.m_tree.Query(QueryCallback, fatAABB);
      }
      __this.m_moveBuffer.length = 0;
      for (var i = 0; i < __this.m_pairCount;) {
         var primaryPair = __this.m_pairBuffer[i];
         var userDataA = __this.m_tree.GetUserData(primaryPair.proxyA);
         var userDataB = __this.m_tree.GetUserData(primaryPair.proxyB);
         callback(userDataA, userDataB);
         ++i;
         while (i < __this.m_pairCount) {
            var pair = __this.m_pairBuffer[i];
            if (pair.proxyA != primaryPair.proxyA || pair.proxyB != primaryPair.proxyB) {
               break;
            }++i;
         }
      }
   }
   b2DynamicTreeBroadPhase.prototype.Query = function (callback, aabb) {
      this.m_tree.Query(callback, aabb);
   }
   b2DynamicTreeBroadPhase.prototype.RayCast = function (callback, input) {
      this.m_tree.RayCast(callback, input);
   }
   b2DynamicTreeBroadPhase.prototype.Validate = function () {}
   b2DynamicTreeBroadPhase.prototype.Rebalance = function (iterations) {
      if (iterations === undefined) iterations = 0;
      this.m_tree.Rebalance(iterations);
   }
   b2DynamicTreeBroadPhase.prototype.BufferMove = function (proxy) {
      this.m_moveBuffer[this.m_moveBuffer.length] = proxy;
   }
   b2DynamicTreeBroadPhase.prototype.UnBufferMove = function (proxy) {
      var i = parseInt(this.m_moveBuffer.indexOf(proxy));
      this.m_moveBuffer.splice(i, 1);
   }
   b2DynamicTreeBroadPhase.prototype.ComparePairs = function (pair1, pair2) {
      return 0;
   }
   b2DynamicTreeBroadPhase.__implements = {};
   b2DynamicTreeBroadPhase.__implements[IBroadPhase] = true;




   b2DynamicTreeNode.b2DynamicTreeNode = function () {
      this.aabb = new b2AABB();
   };
   b2DynamicTreeNode.prototype.IsLeaf = function () {
      return this.child1 == null;
   }




   b2DynamicTreePair.b2DynamicTreePair = function () {};



   b2Manifold.b2Manifold = function () {
      this.m_pointCount = 0;
   };
   b2Manifold.prototype.b2Manifold = function () {
      this.m_points = new Vector(b2Settings.b2_maxManifoldPoints);
      for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
         this.m_points[i] = new b2ManifoldPoint();
      }
      this.m_localPlaneNormal = new b2Vec2();
      this.m_localPoint = new b2Vec2();
   }
   b2Manifold.prototype.Reset = function () {
      for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
         ((this.m_points[i] instanceof b2ManifoldPoint ? this.m_points[i] : null)).Reset();
      }
      this.m_localPlaneNormal.SetZero();
      this.m_localPoint.SetZero();
      this.m_type = 0;
      this.m_pointCount = 0;
   }
   b2Manifold.prototype.Set = function (m) {
      this.m_pointCount = m.m_pointCount;
      for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
         ((this.m_points[i] instanceof b2ManifoldPoint ? this.m_points[i] : null)).Set(m.m_points[i]);
      }
      this.m_localPlaneNormal.SetV(m.m_localPlaneNormal);
      this.m_localPoint.SetV(m.m_localPoint);
      this.m_type = m.m_type;
   }
   b2Manifold.prototype.Copy = function () {
      var copy = new b2Manifold();
      copy.Set(this);
      return copy;
   }
   Box2D.postDefs.push(function () {
      Box2D.Collision.b2Manifold.e_circles = 0x0001;
      Box2D.Collision.b2Manifold.e_faceA = 0x0002;
      Box2D.Collision.b2Manifold.e_faceB = 0x0004;
   });



   b2ManifoldPoint.b2ManifoldPoint = function () {
      this.m_localPoint = new b2Vec2();
      this.m_id = new b2ContactID();
   };
   b2ManifoldPoint.prototype.b2ManifoldPoint = function () {
      this.Reset();
   }
   b2ManifoldPoint.prototype.Reset = function () {
      this.m_localPoint.SetZero();
      this.m_normalImpulse = 0.0;
      this.m_tangentImpulse = 0.0;
      this.m_id.key = 0;
   }
   b2ManifoldPoint.prototype.Set = function (m) {
      this.m_localPoint.SetV(m.m_localPoint);
      this.m_normalImpulse = m.m_normalImpulse;
      this.m_tangentImpulse = m.m_tangentImpulse;
      this.m_id.Set(m.m_id);
   }



   b2Point.b2Point = function () {
      this.p = new b2Vec2();
   };
   b2Point.prototype.Support = function (xf, vX, vY) {
      if (vX === undefined) vX = 0;
      if (vY === undefined) vY = 0;
      return this.p;
   }
   b2Point.prototype.GetFirstVertex = function (xf) {
      return this.p;
   }




   b2RayCastInput.b2RayCastInput = function () {
      this.p1 = new b2Vec2();
      this.p2 = new b2Vec2();
   };
   b2RayCastInput.prototype.b2RayCastInput = function (p1, p2, maxFraction) {
      if (p1 === undefined) p1 = null;
      if (p2 === undefined) p2 = null;
      if (maxFraction === undefined) maxFraction = 1;
      if (p1) this.p1.SetV(p1);
      if (p2) this.p2.SetV(p2);
      this.maxFraction = maxFraction;
   }



   b2RayCastOutput.b2RayCastOutput = function () {
      this.normal = new b2Vec2();
   };



   b2Segment.b2Segment = function () {
      this.p1 = new b2Vec2();
      this.p2 = new b2Vec2();
   };
   b2Segment.prototype.TestSegment = function (lambda, normal, segment, maxLambda) {
      if (maxLambda === undefined) maxLambda = 0;
      var s = segment.p1;
      var rX = segment.p2.x - s.x;
      var rY = segment.p2.y - s.y;
      var dX = this.p2.x - this.p1.x;
      var dY = this.p2.y - this.p1.y;
      var nX = dY;
      var nY = (-dX);
      var k_slop = 100.0 * Number.MIN_VALUE;
      var denom = (-(rX * nX + rY * nY));
      if (denom > k_slop) {
         var bX = s.x - this.p1.x;
         var bY = s.y - this.p1.y;
         var a = (bX * nX + bY * nY);
         if (0.0 <= a && a <= maxLambda * denom) {
            var mu2 = (-rX * bY) + rY * bX;
            if ((-k_slop * denom) <= mu2 && mu2 <= denom * (1.0 + k_slop)) {
               a /= denom;
               var nLen = Math.sqrt(nX * nX + nY * nY);
               nX /= nLen;
               nY /= nLen;
               lambda[0] = a;
               normal.Set(nX, nY);
               return true;
            }
         }
      }
      return false;
   }
   b2Segment.prototype.Extend = function (aabb) {
      this.ExtendForward(aabb);
      this.ExtendBackward(aabb);
   }
   b2Segment.prototype.ExtendForward = function (aabb) {
      var dX = this.p2.x - this.p1.x;
      var dY = this.p2.y - this.p1.y;
      var lambda = Math.min(dX > 0 ? (aabb.upperBound.x - this.p1.x) / dX : dX < 0 ? (aabb.lowerBound.x - this.p1.x) / dX : Number.POSITIVE_INFINITY,
      dY > 0 ? (aabb.upperBound.y - this.p1.y) / dY : dY < 0 ? (aabb.lowerBound.y - this.p1.y) / dY : Number.POSITIVE_INFINITY);
      this.p2.x = this.p1.x + dX * lambda;
      this.p2.y = this.p1.y + dY * lambda;
   }
   b2Segment.prototype.ExtendBackward = function (aabb) {
      var dX = (-this.p2.x) + this.p1.x;
      var dY = (-this.p2.y) + this.p1.y;
      var lambda = Math.min(dX > 0 ? (aabb.upperBound.x - this.p2.x) / dX : dX < 0 ? (aabb.lowerBound.x - this.p2.x) / dX : Number.POSITIVE_INFINITY,
      dY > 0 ? (aabb.upperBound.y - this.p2.y) / dY : dY < 0 ? (aabb.lowerBound.y - this.p2.y) / dY : Number.POSITIVE_INFINITY);
      this.p1.x = this.p2.x + dX * lambda;
      this.p1.y = this.p2.y + dY * lambda;
   }



   b2SeparationFunction.b2SeparationFunction = function () {
      this.m_localPoint = new b2Vec2();
      this.m_axis = new b2Vec2();
   };
   b2SeparationFunction.prototype.Initialize = function (cache, proxyA, transformA, proxyB, transformB) {
      this.m_proxyA = proxyA;
      this.m_proxyB = proxyB;
      var count = parseInt(cache.count);
      b2Settings.b2Assert(0 < count && count < 3);
      var localPointA;
      var localPointA1;
      var localPointA2;
      var localPointB;
      var localPointB1;
      var localPointB2;
      var pointAX = 0;
      var pointAY = 0;
      var pointBX = 0;
      var pointBY = 0;
      var normalX = 0;
      var normalY = 0;
      var tMat;
      var tVec;
      var s = 0;
      var sgn = 0;
      if (count == 1) {
         this.m_type = b2SeparationFunction.e_points;
         localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
         localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
         tVec = localPointA;
         tMat = transformA.R;
         pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
         pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
         tVec = localPointB;
         tMat = transformB.R;
         pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
         pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
         this.m_axis.x = pointBX - pointAX;
         this.m_axis.y = pointBY - pointAY;
         this.m_axis.Normalize();
      }
      else if (cache.indexB[0] == cache.indexB[1]) {
         this.m_type = b2SeparationFunction.e_faceA;
         localPointA1 = this.m_proxyA.GetVertex(cache.indexA[0]);
         localPointA2 = this.m_proxyA.GetVertex(cache.indexA[1]);
         localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
         this.m_localPoint.x = 0.5 * (localPointA1.x + localPointA2.x);
         this.m_localPoint.y = 0.5 * (localPointA1.y + localPointA2.y);
         this.m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointA2, localPointA1), 1.0);
         this.m_axis.Normalize();
         tVec = this.m_axis;
         tMat = transformA.R;
         normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
         normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
         tVec = this.m_localPoint;
         tMat = transformA.R;
         pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
         pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
         tVec = localPointB;
         tMat = transformB.R;
         pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
         pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
         s = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY;
         if (s < 0.0) {
            this.m_axis.NegativeSelf();
         }
      }
      else if (cache.indexA[0] == cache.indexA[0]) {
         this.m_type = b2SeparationFunction.e_faceB;
         localPointB1 = this.m_proxyB.GetVertex(cache.indexB[0]);
         localPointB2 = this.m_proxyB.GetVertex(cache.indexB[1]);
         localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
         this.m_localPoint.x = 0.5 * (localPointB1.x + localPointB2.x);
         this.m_localPoint.y = 0.5 * (localPointB1.y + localPointB2.y);
         this.m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointB2, localPointB1), 1.0);
         this.m_axis.Normalize();
         tVec = this.m_axis;
         tMat = transformB.R;
         normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
         normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
         tVec = this.m_localPoint;
         tMat = transformB.R;
         pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
         pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
         tVec = localPointA;
         tMat = transformA.R;
         pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
         pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
         s = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY;
         if (s < 0.0) {
            this.m_axis.NegativeSelf();
         }
      }
      else {
         localPointA1 = this.m_proxyA.GetVertex(cache.indexA[0]);
         localPointA2 = this.m_proxyA.GetVertex(cache.indexA[1]);
         localPointB1 = this.m_proxyB.GetVertex(cache.indexB[0]);
         localPointB2 = this.m_proxyB.GetVertex(cache.indexB[1]);
         var pA = b2Math.MulX(transformA, localPointA);
         var dA = b2Math.MulMV(transformA.R, b2Math.SubtractVV(localPointA2, localPointA1));
         var pB = b2Math.MulX(transformB, localPointB);
         var dB = b2Math.MulMV(transformB.R, b2Math.SubtractVV(localPointB2, localPointB1));
         var a = dA.x * dA.x + dA.y * dA.y;
         var e = dB.x * dB.x + dB.y * dB.y;
         var r = b2Math.SubtractVV(dB, dA);
         var c = dA.x * r.x + dA.y * r.y;
         var f = dB.x * r.x + dB.y * r.y;
         var b = dA.x * dB.x + dA.y * dB.y;
         var denom = a * e - b * b;
         s = 0.0;
         if (denom != 0.0) {
            s = b2Math.Clamp((b * f - c * e) / denom, 0.0, 1.0);
         }
         var t = (b * s + f) / e;
         if (t < 0.0) {
            t = 0.0;
            s = b2Math.Clamp((b - c) / a, 0.0, 1.0);
         }
         localPointA = new b2Vec2();
         localPointA.x = localPointA1.x + s * (localPointA2.x - localPointA1.x);
         localPointA.y = localPointA1.y + s * (localPointA2.y - localPointA1.y);
         localPointB = new b2Vec2();
         localPointB.x = localPointB1.x + s * (localPointB2.x - localPointB1.x);
         localPointB.y = localPointB1.y + s * (localPointB2.y - localPointB1.y);
         if (s == 0.0 || s == 1.0) {
            this.m_type = b2SeparationFunction.e_faceB;
            this.m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointB2, localPointB1), 1.0);
            this.m_axis.Normalize();
            this.m_localPoint = localPointB;
            tVec = this.m_axis;
            tMat = transformB.R;
            normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            tVec = this.m_localPoint;
            tMat = transformB.R;
            pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            tVec = localPointA;
            tMat = transformA.R;
            pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            sgn = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY;
            if (s < 0.0) {
               this.m_axis.NegativeSelf();
            }
         }
         else {
            this.m_type = b2SeparationFunction.e_faceA;
            this.m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointA2, localPointA1), 1.0);
            this.m_localPoint = localPointA;
            tVec = this.m_axis;
            tMat = transformA.R;
            normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            tVec = this.m_localPoint;
            tMat = transformA.R;
            pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            tVec = localPointB;
            tMat = transformB.R;
            pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            sgn = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY;
            if (s < 0.0) {
               this.m_axis.NegativeSelf();
            }
         }
      }
   }
   b2SeparationFunction.prototype.Evaluate = function (transformA, transformB) {
      var axisA;
      var axisB;
      var localPointA;
      var localPointB;
      var pointA;
      var pointB;
      var seperation = 0;
      var normal;
      switch (this.m_type) {
      case b2SeparationFunction.e_points:
         {
            axisA = b2Math.MulTMV(transformA.R, this.m_axis);
            axisB = b2Math.MulTMV(transformB.R, this.m_axis.GetNegative());
            localPointA = this.m_proxyA.GetSupportVertex(axisA);
            localPointB = this.m_proxyB.GetSupportVertex(axisB);
            pointA = b2Math.MulX(transformA, localPointA);
            pointB = b2Math.MulX(transformB, localPointB);
            seperation = (pointB.x - pointA.x) * this.m_axis.x + (pointB.y - pointA.y) * this.m_axis.y;
            return seperation;
         }
      case b2SeparationFunction.e_faceA:
         {
            normal = b2Math.MulMV(transformA.R, this.m_axis);
            pointA = b2Math.MulX(transformA, this.m_localPoint);
            axisB = b2Math.MulTMV(transformB.R, normal.GetNegative());
            localPointB = this.m_proxyB.GetSupportVertex(axisB);
            pointB = b2Math.MulX(transformB, localPointB);
            seperation = (pointB.x - pointA.x) * normal.x + (pointB.y - pointA.y) * normal.y;
            return seperation;
         }
      case b2SeparationFunction.e_faceB:
         {
            normal = b2Math.MulMV(transformB.R, this.m_axis);
            pointB = b2Math.MulX(transformB, this.m_localPoint);
            axisA = b2Math.MulTMV(transformA.R, normal.GetNegative());
            localPointA = this.m_proxyA.GetSupportVertex(axisA);
            pointA = b2Math.MulX(transformA, localPointA);
            seperation = (pointA.x - pointB.x) * normal.x + (pointA.y - pointB.y) * normal.y;
            return seperation;
         }
      default:
         b2Settings.b2Assert(false);
         return 0.0;
      }
   }
   Box2D.postDefs.push(function () {
      Box2D.Collision.b2SeparationFunction.e_points = 0x01;
      Box2D.Collision.b2SeparationFunction.e_faceA = 0x02;
      Box2D.Collision.b2SeparationFunction.e_faceB = 0x04;
   });




   b2Simplex.b2Simplex = function () {
      this.m_v1 = new b2SimplexVertex();
      this.m_v2 = new b2SimplexVertex();
      this.m_v3 = new b2SimplexVertex();
      this.m_vertices = new Vector(3);
   };
   b2Simplex.prototype.b2Simplex = function () {
      this.m_vertices[0] = this.m_v1;
      this.m_vertices[1] = this.m_v2;
      this.m_vertices[2] = this.m_v3;
   }
   b2Simplex.prototype.ReadCache = function (cache, proxyA, transformA, proxyB, transformB) {
      b2Settings.b2Assert(0 <= cache.count && cache.count <= 3);
      var wALocal;
      var wBLocal;
      this.m_count = cache.count;
      var vertices = this.m_vertices;
      for (var i = 0; i < this.m_count; i++) {
         var v = vertices[i];
         v.indexA = cache.indexA[i];
         v.indexB = cache.indexB[i];
         wALocal = proxyA.GetVertex(v.indexA);
         wBLocal = proxyB.GetVertex(v.indexB);
         v.wA = b2Math.MulX(transformA, wALocal);
         v.wB = b2Math.MulX(transformB, wBLocal);
         v.w = b2Math.SubtractVV(v.wB, v.wA);
         v.a = 0;
      }
      if (this.m_count > 1) {
         var metric1 = cache.metric;
         var metric2 = this.GetMetric();
         if (metric2 < .5 * metric1 || 2.0 * metric1 < metric2 || metric2 < Number.MIN_VALUE) {
            this.m_count = 0;
         }
      }
      if (this.m_count == 0) {
         v = vertices[0];
         v.indexA = 0;
         v.indexB = 0;
         wALocal = proxyA.GetVertex(0);
         wBLocal = proxyB.GetVertex(0);
         v.wA = b2Math.MulX(transformA, wALocal);
         v.wB = b2Math.MulX(transformB, wBLocal);
         v.w = b2Math.SubtractVV(v.wB, v.wA);
         this.m_count = 1;
      }
   }
   b2Simplex.prototype.WriteCache = function (cache) {
      cache.metric = this.GetMetric();
      cache.count = Box2D.parseUInt(this.m_count);
      var vertices = this.m_vertices;
      for (var i = 0; i < this.m_count; i++) {
         cache.indexA[i] = Box2D.parseUInt(vertices[i].indexA);
         cache.indexB[i] = Box2D.parseUInt(vertices[i].indexB);
      }
   }
   b2Simplex.prototype.GetSearchDirection = function () {
      switch (this.m_count) {
      case 1:
         return this.m_v1.w.GetNegative();
      case 2:
         {
            var e12 = b2Math.SubtractVV(this.m_v2.w, this.m_v1.w);
            var sgn = b2Math.CrossVV(e12, this.m_v1.w.GetNegative());
            if (sgn > 0.0) {
               return b2Math.CrossFV(1.0, e12);
            }
            else {
               return b2Math.CrossVF(e12, 1.0);
            }
         }
      default:
         b2Settings.b2Assert(false);
         return new b2Vec2();
      }
   }
   b2Simplex.prototype.GetClosestPoint = function () {
      switch (this.m_count) {
      case 0:
         b2Settings.b2Assert(false);
         return new b2Vec2();
      case 1:
         return this.m_v1.w;
      case 2:
         return new b2Vec2(this.m_v1.a * this.m_v1.w.x + this.m_v2.a * this.m_v2.w.x, this.m_v1.a * this.m_v1.w.y + this.m_v2.a * this.m_v2.w.y);
      default:
         b2Settings.b2Assert(false);
         return new b2Vec2();
      }
   }
   b2Simplex.prototype.GetWitnessPoints = function (pA, pB) {
      switch (this.m_count) {
      case 0:
         b2Settings.b2Assert(false);
         break;
      case 1:
         pA.SetV(this.m_v1.wA);
         pB.SetV(this.m_v1.wB);
         break;
      case 2:
         pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x;
         pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y;
         pB.x = this.m_v1.a * this.m_v1.wB.x + this.m_v2.a * this.m_v2.wB.x;
         pB.y = this.m_v1.a * this.m_v1.wB.y + this.m_v2.a * this.m_v2.wB.y;
         break;
      case 3:
         pB.x = pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x + this.m_v3.a * this.m_v3.wA.x;
         pB.y = pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y + this.m_v3.a * this.m_v3.wA.y;
         break;
      default:
         b2Settings.b2Assert(false);
         break;
      }
   }
   b2Simplex.prototype.GetMetric = function () {
      switch (this.m_count) {
      case 0:
         b2Settings.b2Assert(false);
         return 0.0;
      case 1:
         return 0.0;
      case 2:
         return b2Math.SubtractVV(this.m_v1.w, this.m_v2.w).Length();
      case 3:
         return b2Math.CrossVV(b2Math.SubtractVV(this.m_v2.w, this.m_v1.w), b2Math.SubtractVV(this.m_v3.w, this.m_v1.w));
      default:
         b2Settings.b2Assert(false);
         return 0.0;
      }
   }
   b2Simplex.prototype.Solve2 = function () {
      var w1 = this.m_v1.w;
      var w2 = this.m_v2.w;
      var e12 = b2Math.SubtractVV(w2, w1);
      var d12_2 = (-(w1.x * e12.x + w1.y * e12.y));
      if (d12_2 <= 0.0) {
         this.m_v1.a = 1.0;
         this.m_count = 1;
         return;
      }
      var d12_1 = (w2.x * e12.x + w2.y * e12.y);
      if (d12_1 <= 0.0) {
         this.m_v2.a = 1.0;
         this.m_count = 1;
         this.m_v1.Set(this.m_v2);
         return;
      }
      var inv_d12 = 1.0 / (d12_1 + d12_2);
      this.m_v1.a = d12_1 * inv_d12;
      this.m_v2.a = d12_2 * inv_d12;
      this.m_count = 2;
   }
   b2Simplex.prototype.Solve3 = function () {
      var w1 = this.m_v1.w;
      var w2 = this.m_v2.w;
      var w3 = this.m_v3.w;
      var e12 = b2Math.SubtractVV(w2, w1);
      var w1e12 = b2Math.Dot(w1, e12);
      var w2e12 = b2Math.Dot(w2, e12);
      var d12_1 = w2e12;
      var d12_2 = (-w1e12);
      var e13 = b2Math.SubtractVV(w3, w1);
      var w1e13 = b2Math.Dot(w1, e13);
      var w3e13 = b2Math.Dot(w3, e13);
      var d13_1 = w3e13;
      var d13_2 = (-w1e13);
      var e23 = b2Math.SubtractVV(w3, w2);
      var w2e23 = b2Math.Dot(w2, e23);
      var w3e23 = b2Math.Dot(w3, e23);
      var d23_1 = w3e23;
      var d23_2 = (-w2e23);
      var n123 = b2Math.CrossVV(e12, e13);
      var d123_1 = n123 * b2Math.CrossVV(w2, w3);
      var d123_2 = n123 * b2Math.CrossVV(w3, w1);
      var d123_3 = n123 * b2Math.CrossVV(w1, w2);
      if (d12_2 <= 0.0 && d13_2 <= 0.0) {
         this.m_v1.a = 1.0;
         this.m_count = 1;
         return;
      }
      if (d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0) {
         var inv_d12 = 1.0 / (d12_1 + d12_2);
         this.m_v1.a = d12_1 * inv_d12;
         this.m_v2.a = d12_2 * inv_d12;
         this.m_count = 2;
         return;
      }
      if (d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0) {
         var inv_d13 = 1.0 / (d13_1 + d13_2);
         this.m_v1.a = d13_1 * inv_d13;
         this.m_v3.a = d13_2 * inv_d13;
         this.m_count = 2;
         this.m_v2.Set(this.m_v3);
         return;
      }
      if (d12_1 <= 0.0 && d23_2 <= 0.0) {
         this.m_v2.a = 1.0;
         this.m_count = 1;
         this.m_v1.Set(this.m_v2);
         return;
      }
      if (d13_1 <= 0.0 && d23_1 <= 0.0) {
         this.m_v3.a = 1.0;
         this.m_count = 1;
         this.m_v1.Set(this.m_v3);
         return;
      }
      if (d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0) {
         var inv_d23 = 1.0 / (d23_1 + d23_2);
         this.m_v2.a = d23_1 * inv_d23;
         this.m_v3.a = d23_2 * inv_d23;
         this.m_count = 2;
         this.m_v1.Set(this.m_v3);
         return;
      }
      var inv_d123 = 1.0 / (d123_1 + d123_2 + d123_3);
      this.m_v1.a = d123_1 * inv_d123;
      this.m_v2.a = d123_2 * inv_d123;
      this.m_v3.a = d123_3 * inv_d123;
      this.m_count = 3;
   }



   b2SimplexCache.b2SimplexCache = function () {
      this.indexA = new Vector_a2j_Number(3);
      this.indexB = new Vector_a2j_Number(3);
   };



   b2SimplexVertex.b2SimplexVertex = function () {};
   b2SimplexVertex.prototype.Set = function (other) {
      this.wA.SetV(other.wA);
      this.wB.SetV(other.wB);
      this.w.SetV(other.w);
      this.a = other.a;
      this.indexA = other.indexA;
      this.indexB = other.indexB;
   }



   b2TimeOfImpact.b2TimeOfImpact = function () {};
   b2TimeOfImpact.TimeOfImpact = function (input) {
      ++b2TimeOfImpact.b2_toiCalls;
      var proxyA = input.proxyA;
      var proxyB = input.proxyB;
      var sweepA = input.sweepA;
      var sweepB = input.sweepB;
      b2Settings.b2Assert(sweepA.t0 == sweepB.t0);
      b2Settings.b2Assert(1.0 - sweepA.t0 > Number.MIN_VALUE);
      var radius = proxyA.m_radius + proxyB.m_radius;
      var tolerance = input.tolerance;
      var alpha = 0.0;
      var k_maxIterations = 1000;
      var iter = 0;
      var target = 0.0;
      b2TimeOfImpact.s_cache.count = 0;
      b2TimeOfImpact.s_distanceInput.useRadii = false;
      for (;;) {
         sweepA.GetTransform(b2TimeOfImpact.s_xfA, alpha);
         sweepB.GetTransform(b2TimeOfImpact.s_xfB, alpha);
         b2TimeOfImpact.s_distanceInput.proxyA = proxyA;
         b2TimeOfImpact.s_distanceInput.proxyB = proxyB;
         b2TimeOfImpact.s_distanceInput.transformA = b2TimeOfImpact.s_xfA;
         b2TimeOfImpact.s_distanceInput.transformB = b2TimeOfImpact.s_xfB;
         b2Distance.Distance(b2TimeOfImpact.s_distanceOutput, b2TimeOfImpact.s_cache, b2TimeOfImpact.s_distanceInput);
         if (b2TimeOfImpact.s_distanceOutput.distance <= 0.0) {
            alpha = 1.0;
            break;
         }
         b2TimeOfImpact.s_fcn.Initialize(b2TimeOfImpact.s_cache, proxyA, b2TimeOfImpact.s_xfA, proxyB, b2TimeOfImpact.s_xfB);
         var separation = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB);
         if (separation <= 0.0) {
            alpha = 1.0;
            break;
         }
         if (iter == 0) {
            if (separation > radius) {
               target = b2Math.Max(radius - tolerance, 0.75 * radius);
            }
            else {
               target = b2Math.Max(separation - tolerance, 0.02 * radius);
            }
         }
         if (separation - target < 0.5 * tolerance) {
            if (iter == 0) {
               alpha = 1.0;
               break;
            }
            break;
         }
         var newAlpha = alpha; {
            var x1 = alpha;
            var x2 = 1.0;
            var f1 = separation;
            sweepA.GetTransform(b2TimeOfImpact.s_xfA, x2);
            sweepB.GetTransform(b2TimeOfImpact.s_xfB, x2);
            var f2 = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB);
            if (f2 >= target) {
               alpha = 1.0;
               break;
            }
            var rootIterCount = 0;
            for (;;) {
               var x = 0;
               if (rootIterCount & 1) {
                  x = x1 + (target - f1) * (x2 - x1) / (f2 - f1);
               }
               else {
                  x = 0.5 * (x1 + x2);
               }
               sweepA.GetTransform(b2TimeOfImpact.s_xfA, x);
               sweepB.GetTransform(b2TimeOfImpact.s_xfB, x);
               var f = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB);
               if (b2Math.Abs(f - target) < 0.025 * tolerance) {
                  newAlpha = x;
                  break;
               }
               if (f > target) {
                  x1 = x;
                  f1 = f;
               }
               else {
                  x2 = x;
                  f2 = f;
               }++rootIterCount;
               ++b2TimeOfImpact.b2_toiRootIters;
               if (rootIterCount == 50) {
                  break;
               }
            }
            b2TimeOfImpact.b2_toiMaxRootIters = b2Math.Max(b2TimeOfImpact.b2_toiMaxRootIters, rootIterCount);
         }
         if (newAlpha < (1.0 + 100.0 * Number.MIN_VALUE) * alpha) {
            break;
         }
         alpha = newAlpha;
         iter++;
         ++b2TimeOfImpact.b2_toiIters;
         if (iter == k_maxIterations) {
            break;
         }
      }
      b2TimeOfImpact.b2_toiMaxIters = b2Math.Max(b2TimeOfImpact.b2_toiMaxIters, iter);
      return alpha;
   }
   Box2D.postDefs.push(function () {
      Box2D.Collision.b2TimeOfImpact.b2_toiCalls = 0;
      Box2D.Collision.b2TimeOfImpact.b2_toiIters = 0;
      Box2D.Collision.b2TimeOfImpact.b2_toiMaxIters = 0;
      Box2D.Collision.b2TimeOfImpact.b2_toiRootIters = 0;
      Box2D.Collision.b2TimeOfImpact.b2_toiMaxRootIters = 0;
      Box2D.Collision.b2TimeOfImpact.s_cache = new b2SimplexCache();
      Box2D.Collision.b2TimeOfImpact.s_distanceInput = new b2DistanceInput();
      Box2D.Collision.b2TimeOfImpact.s_xfA = new b2Transform();
      Box2D.Collision.b2TimeOfImpact.s_xfB = new b2Transform();
      Box2D.Collision.b2TimeOfImpact.s_fcn = new b2SeparationFunction();
      Box2D.Collision.b2TimeOfImpact.s_distanceOutput = new b2DistanceOutput();
   });



   b2TOIInput.b2TOIInput = function () {
      this.proxyA = new b2DistanceProxy();
      this.proxyB = new b2DistanceProxy();
      this.sweepA = new b2Sweep();
      this.sweepB = new b2Sweep();
   };



   b2WorldManifold.b2WorldManifold = function () {
      this.m_normal = new b2Vec2();
   };
   b2WorldManifold.prototype.b2WorldManifold = function () {
      this.m_points = new Vector(b2Settings.b2_maxManifoldPoints);
      for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
         this.m_points[i] = new b2Vec2();
      }
   }
   b2WorldManifold.prototype.Initialize = function (manifold, xfA, radiusA, xfB, radiusB) {
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
            }
            else {
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

   


   ClipVertex.ClipVertex = function () {
      this.v = new b2Vec2();
      this.id = new b2ContactID();
   };
   ClipVertex.prototype.Set = function (other) {
      this.v.SetV(other.v);
      this.id.Set(other.id);
   }

   

   Features.Features = function () {};
   Object.defineProperty(Features.prototype, 'referenceEdge', {
      enumerable: false,
      configurable: true,
      get: function () {
         return this._referenceEdge;
      }
   });
   Object.defineProperty(Features.prototype, 'referenceEdge', {
      enumerable: false,
      configurable: true,
      set: function (value) {
         if (value === undefined) value = 0;
         this._referenceEdge = value;
         this._m_id._key = (this._m_id._key & 0xffffff00) | (this._referenceEdge & 0x000000ff);
      }
   });
   Object.defineProperty(Features.prototype, 'incidentEdge', {
      enumerable: false,
      configurable: true,
      get: function () {
         return this._incidentEdge;
      }
   });
   Object.defineProperty(Features.prototype, 'incidentEdge', {
      enumerable: false,
      configurable: true,
      set: function (value) {
         if (value === undefined) value = 0;
         this._incidentEdge = value;
         this._m_id._key = (this._m_id._key & 0xffff00ff) | ((this._incidentEdge << 8) & 0x0000ff00);
      }
   });
   Object.defineProperty(Features.prototype, 'incidentVertex', {
      enumerable: false,
      configurable: true,
      get: function () {
         return this._incidentVertex;
      }
   });
   Object.defineProperty(Features.prototype, 'incidentVertex', {
      enumerable: false,
      configurable: true,
      set: function (value) {
         if (value === undefined) value = 0;
         this._incidentVertex = value;
         this._m_id._key = (this._m_id._key & 0xff00ffff) | ((this._incidentVertex << 16) & 0x00ff0000);
      }
   });
   Object.defineProperty(Features.prototype, 'flip', {
      enumerable: false,
      configurable: true,
      get: function () {
         return this._flip;
      }
   });
   Object.defineProperty(Features.prototype, 'flip', {
      enumerable: false,
      configurable: true,
      set: function (value) {
         if (value === undefined) value = 0;
         this._flip = value;
         this._m_id._key = (this._m_id._key & 0x00ffffff) | ((this._flip << 24) & 0xff000000);
      }
   });
