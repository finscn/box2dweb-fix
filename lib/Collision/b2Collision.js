function b2Collision() {

};
Box2D.Collision.b2Collision = b2Collision;

b2Collision.ClipSegmentToLine = function(vOut, vIn, normal, offset) {
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
        } else {
            cv2 = vIn[1];
            cv.id = cv2.id;
        }++numOut;
    }
    return numOut;
}
b2Collision.EdgeSeparation = function(poly1, xf1, edge1, poly2, xf2) {
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
b2Collision.FindMaxSeparation = function(edgeIndex, poly1, xf1, poly2, xf2) {
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
    } else if (sNext > s) {
        increment = 1;
        bestEdge = nextEdge;
        bestSeparation = sNext;
    } else {
        edgeIndex[0] = edge;
        return s;
    }
    while (true) {
        if (increment == (-1)) edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
        else edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;
        s = b2Collision.EdgeSeparation(poly1, xf1, edge, poly2, xf2);
        if (s > bestSeparation) {
            bestEdge = edge;
            bestSeparation = s;
        } else {
            break;
        }
    }
    edgeIndex[0] = bestEdge;
    return bestSeparation;
}
b2Collision.FindIncidentEdge = function(c, poly1, xf1, edge1, poly2, xf2) {
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
b2Collision.MakeClipPointVector = function() {
    var r = new Vector(2);
    r[0] = new ClipVertex();
    r[1] = new ClipVertex();
    return r;
}
b2Collision.CollidePolygons = function(manifold, polyA, xfA, polyB, xfB) {
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
    } else {
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
    } else {
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
b2Collision.CollideCircles = function(manifold, circle1, xf1, circle2, xf2) {
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
b2Collision.CollidePolygonAndCircle = function(manifold, polygon, xf1, circle, xf2) {
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
    } else if (u2 <= 0) {
        if ((cLocalX - v2.x) * (cLocalX - v2.x) + (cLocalY - v2.y) * (cLocalY - v2.y) > radius * radius) return;
        manifold.m_pointCount = 1;
        manifold.m_type = b2Manifold.e_faceA;
        manifold.m_localPlaneNormal.x = cLocalX - v2.x;
        manifold.m_localPlaneNormal.y = cLocalY - v2.y;
        manifold.m_localPlaneNormal.Normalize();
        manifold.m_localPoint.SetV(v2);
        manifold.m_points[0].m_localPoint.SetV(circle.m_p);
        manifold.m_points[0].m_id.key = 0;
    } else {
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
b2Collision.TestOverlap = function(a, b) {
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
Box2D.postDefs.push(function() {
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