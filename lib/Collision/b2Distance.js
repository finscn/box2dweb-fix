function b2Distance() {

};
Box2D.Collision.b2Distance = b2Distance;

b2Distance.Distance = function(output, cache, input) {
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
        } else {
            p = new b2Vec2();
            p.x = .5 * (output.pointA.x + output.pointB.x);
            p.y = .5 * (output.pointA.y + output.pointB.y);
            output.pointA.x = output.pointB.x = p.x;
            output.pointA.y = output.pointB.y = p.y;
            output.distance = 0.0;
        }
    }
}
Box2D.postDefs.push(function() {
    Box2D.Collision.b2Distance.s_simplex = new b2Simplex();
    Box2D.Collision.b2Distance.s_saveA = new Vector_a2j_Number(3);
    Box2D.Collision.b2Distance.s_saveB = new Vector_a2j_Number(3);
});