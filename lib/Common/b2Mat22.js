function b2Mat22() {
    this.col1 = new b2Vec2();
    this.col2 = new b2Vec2();
    this.SetIdentity();

};
Box2D.Common.Math.b2Mat22 = b2Mat22;


b2Mat22.FromAngle = function(angle) {
    if (angle === undefined) angle = 0;
    var mat = new b2Mat22();
    mat.Set(angle);
    return mat;
}
b2Mat22.FromVV = function(c1, c2) {
    var mat = new b2Mat22();
    mat.SetVV(c1, c2);
    return mat;
}
b2Mat22.prototype.Set = function(angle) {
    if (angle === undefined) angle = 0;
    var c = Math.cos(angle);
    var s = Math.sin(angle);
    this.col1.x = c;
    this.col2.x = (-s);
    this.col1.y = s;
    this.col2.y = c;
}
b2Mat22.prototype.SetVV = function(c1, c2) {
    this.col1.SetV(c1);
    this.col2.SetV(c2);
}
b2Mat22.prototype.Copy = function() {
    var mat = new b2Mat22();
    mat.SetM(this);
    return mat;
}
b2Mat22.prototype.SetM = function(m) {
    this.col1.SetV(m.col1);
    this.col2.SetV(m.col2);
}
b2Mat22.prototype.AddM = function(m) {
    this.col1.x += m.col1.x;
    this.col1.y += m.col1.y;
    this.col2.x += m.col2.x;
    this.col2.y += m.col2.y;
}
b2Mat22.prototype.SetIdentity = function() {
    this.col1.x = 1.0;
    this.col2.x = 0.0;
    this.col1.y = 0.0;
    this.col2.y = 1.0;
}
b2Mat22.prototype.SetZero = function() {
    this.col1.x = 0.0;
    this.col2.x = 0.0;
    this.col1.y = 0.0;
    this.col2.y = 0.0;
}
b2Mat22.prototype.GetAngle = function() {
    return Math.atan2(this.col1.y, this.col1.x);
}
b2Mat22.prototype.GetInverse = function(out) {
    var a = this.col1.x;
    var b = this.col2.x;
    var c = this.col1.y;
    var d = this.col2.y;
    var det = a * d - b * c;
    if (det != 0.0) {
        det = 1.0 / det;
    }
    out.col1.x = det * d;
    out.col2.x = (-det * b);
    out.col1.y = (-det * c);
    out.col2.y = det * a;
    return out;
}
b2Mat22.prototype.Solve = function(out, bX, bY) {
    if (bX === undefined) bX = 0;
    if (bY === undefined) bY = 0;
    var a11 = this.col1.x;
    var a12 = this.col2.x;
    var a21 = this.col1.y;
    var a22 = this.col2.y;
    var det = a11 * a22 - a12 * a21;
    if (det != 0.0) {
        det = 1.0 / det;
    }
    out.x = det * (a22 * bX - a12 * bY);
    out.y = det * (a11 * bY - a21 * bX);
    return out;
}
b2Mat22.prototype.Abs = function() {
    this.col1.Abs();
    this.col2.Abs();
}