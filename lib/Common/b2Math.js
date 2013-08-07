function b2Math() {

};
Box2D.Common.Math.b2Math = b2Math;

b2Math.IsValid = function(x) {
    if (x === undefined) x = 0;
    return isFinite(x);
}
b2Math.Dot = function(a, b) {
    return a.x * b.x + a.y * b.y;
}
b2Math.CrossVV = function(a, b) {
    return a.x * b.y - a.y * b.x;
}
b2Math.CrossVF = function(a, s) {
    if (s === undefined) s = 0;
    var v = new b2Vec2(s * a.y, (-s * a.x));
    return v;
}
b2Math.CrossFV = function(s, a) {
    if (s === undefined) s = 0;
    var v = new b2Vec2((-s * a.y), s * a.x);
    return v;
}
b2Math.MulMV = function(A, v) {
    var u = new b2Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
    return u;
}
b2Math.MulTMV = function(A, v) {
    var u = new b2Vec2(b2Math.Dot(v, A.col1), b2Math.Dot(v, A.col2));
    return u;
}
b2Math.MulX = function(T, v) {
    var a = b2Math.MulMV(T.R, v);
    a.x += T.position.x;
    a.y += T.position.y;
    return a;
}
b2Math.MulXT = function(T, v) {
    var a = b2Math.SubtractVV(v, T.position);
    var tX = (a.x * T.R.col1.x + a.y * T.R.col1.y);
    a.y = (a.x * T.R.col2.x + a.y * T.R.col2.y);
    a.x = tX;
    return a;
}
b2Math.AddVV = function(a, b) {
    var v = new b2Vec2(a.x + b.x, a.y + b.y);
    return v;
}
b2Math.SubtractVV = function(a, b) {
    var v = new b2Vec2(a.x - b.x, a.y - b.y);
    return v;
}
b2Math.Distance = function(a, b) {
    var cX = a.x - b.x;
    var cY = a.y - b.y;
    return Math.sqrt(cX * cX + cY * cY);
}
b2Math.DistanceSquared = function(a, b) {
    var cX = a.x - b.x;
    var cY = a.y - b.y;
    return (cX * cX + cY * cY);
}
b2Math.MulFV = function(s, a) {
    if (s === undefined) s = 0;
    var v = new b2Vec2(s * a.x, s * a.y);
    return v;
}
b2Math.AddMM = function(A, B) {
    var C = b2Mat22.FromVV(b2Math.AddVV(A.col1, B.col1), b2Math.AddVV(A.col2, B.col2));
    return C;
}
b2Math.MulMM = function(A, B) {
    var C = b2Mat22.FromVV(b2Math.MulMV(A, B.col1), b2Math.MulMV(A, B.col2));
    return C;
}
b2Math.MulTMM = function(A, B) {
    var c1 = new b2Vec2(b2Math.Dot(A.col1, B.col1), b2Math.Dot(A.col2, B.col1));
    var c2 = new b2Vec2(b2Math.Dot(A.col1, B.col2), b2Math.Dot(A.col2, B.col2));
    var C = b2Mat22.FromVV(c1, c2);
    return C;
}
b2Math.Abs = function(a) {
    if (a === undefined) a = 0;
    return a > 0.0 ? a : (-a);
}
b2Math.AbsV = function(a) {
    var b = new b2Vec2(b2Math.Abs(a.x), b2Math.Abs(a.y));
    return b;
}
b2Math.AbsM = function(A) {
    var B = b2Mat22.FromVV(b2Math.AbsV(A.col1), b2Math.AbsV(A.col2));
    return B;
}
b2Math.Min = function(a, b) {
    if (a === undefined) a = 0;
    if (b === undefined) b = 0;
    return a < b ? a : b;
}
b2Math.MinV = function(a, b) {
    var c = new b2Vec2(b2Math.Min(a.x, b.x), b2Math.Min(a.y, b.y));
    return c;
}
b2Math.Max = function(a, b) {
    if (a === undefined) a = 0;
    if (b === undefined) b = 0;
    return a > b ? a : b;
}
b2Math.MaxV = function(a, b) {
    var c = new b2Vec2(b2Math.Max(a.x, b.x), b2Math.Max(a.y, b.y));
    return c;
}
b2Math.Clamp = function(a, low, high) {
    if (a === undefined) a = 0;
    if (low === undefined) low = 0;
    if (high === undefined) high = 0;
    return a < low ? low : a > high ? high : a;
}
b2Math.ClampV = function(a, low, high) {
    return b2Math.MaxV(low, b2Math.MinV(a, high));
}
b2Math.Swap = function(a, b) {
    var tmp = a[0];
    a[0] = b[0];
    b[0] = tmp;
}
b2Math.Random = function() {
    return Math.random() * 2 - 1;
}
b2Math.RandomRange = function(lo, hi) {
    if (lo === undefined) lo = 0;
    if (hi === undefined) hi = 0;
    var r = Math.random();
    r = (hi - lo) * r + lo;
    return r;
}
b2Math.NextPowerOfTwo = function(x) {
    if (x === undefined) x = 0;
    x |= (x >> 1) & 0x7FFFFFFF;
    x |= (x >> 2) & 0x3FFFFFFF;
    x |= (x >> 4) & 0x0FFFFFFF;
    x |= (x >> 8) & 0x00FFFFFF;
    x |= (x >> 16) & 0x0000FFFF;
    return x + 1;
}
b2Math.IsPowerOfTwo = function(x) {
    if (x === undefined) x = 0;
    var result = x > 0 && (x & (x - 1)) == 0;
    return result;
}
Box2D.postDefs.push(function() {
    Box2D.Common.Math.b2Math.b2Vec2_zero = new b2Vec2(0.0, 0.0);
    Box2D.Common.Math.b2Math.b2Mat22_identity = b2Mat22.FromVV(new b2Vec2(1.0, 0.0), new b2Vec2(0.0, 1.0));
    Box2D.Common.Math.b2Math.b2Transform_identity = new b2Transform(b2Math.b2Vec2_zero, b2Math.b2Mat22_identity);
});