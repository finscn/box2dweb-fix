function b2Mat33(c1, c2, c3) {
    this.col1 = new b2Vec3();
    this.col2 = new b2Vec3();
    this.col3 = new b2Vec3();
    if (c1 === undefined) c1 = null;
    if (c2 === undefined) c2 = null;
    if (c3 === undefined) c3 = null;
    if (!c1 && !c2 && !c3) {
        this.col1.SetZero();
        this.col2.SetZero();
        this.col3.SetZero();
    } else {
        this.col1.SetV(c1);
        this.col2.SetV(c2);
        this.col3.SetV(c3);
    }
};
Box2D.Common.Math.b2Mat33 = b2Mat33;


b2Mat33.prototype.SetVVV = function(c1, c2, c3) {
    this.col1.SetV(c1);
    this.col2.SetV(c2);
    this.col3.SetV(c3);
}
b2Mat33.prototype.Copy = function() {
    return new b2Mat33(this.col1, this.col2, this.col3);
}
b2Mat33.prototype.SetM = function(m) {
    this.col1.SetV(m.col1);
    this.col2.SetV(m.col2);
    this.col3.SetV(m.col3);
}
b2Mat33.prototype.AddM = function(m) {
    this.col1.x += m.col1.x;
    this.col1.y += m.col1.y;
    this.col1.z += m.col1.z;
    this.col2.x += m.col2.x;
    this.col2.y += m.col2.y;
    this.col2.z += m.col2.z;
    this.col3.x += m.col3.x;
    this.col3.y += m.col3.y;
    this.col3.z += m.col3.z;
}
b2Mat33.prototype.SetIdentity = function() {
    this.col1.x = 1.0;
    this.col2.x = 0.0;
    this.col3.x = 0.0;
    this.col1.y = 0.0;
    this.col2.y = 1.0;
    this.col3.y = 0.0;
    this.col1.z = 0.0;
    this.col2.z = 0.0;
    this.col3.z = 1.0;
}
b2Mat33.prototype.SetZero = function() {
    this.col1.x = 0.0;
    this.col2.x = 0.0;
    this.col3.x = 0.0;
    this.col1.y = 0.0;
    this.col2.y = 0.0;
    this.col3.y = 0.0;
    this.col1.z = 0.0;
    this.col2.z = 0.0;
    this.col3.z = 0.0;
}
b2Mat33.prototype.Solve22 = function(out, bX, bY) {
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
b2Mat33.prototype.Solve33 = function(out, bX, bY, bZ) {
    if (bX === undefined) bX = 0;
    if (bY === undefined) bY = 0;
    if (bZ === undefined) bZ = 0;
    var a11 = this.col1.x;
    var a21 = this.col1.y;
    var a31 = this.col1.z;
    var a12 = this.col2.x;
    var a22 = this.col2.y;
    var a32 = this.col2.z;
    var a13 = this.col3.x;
    var a23 = this.col3.y;
    var a33 = this.col3.z;
    var det = a11 * (a22 * a33 - a32 * a23) + a21 * (a32 * a13 - a12 * a33) + a31 * (a12 * a23 - a22 * a13);
    if (det != 0.0) {
        det = 1.0 / det;
    }
    out.x = det * (bX * (a22 * a33 - a32 * a23) + bY * (a32 * a13 - a12 * a33) + bZ * (a12 * a23 - a22 * a13));
    out.y = det * (a11 * (bY * a33 - bZ * a23) + a21 * (bZ * a13 - bX * a33) + a31 * (bX * a23 - bY * a13));
    out.z = det * (a11 * (a22 * bZ - a32 * bY) + a21 * (a32 * bX - a12 * bZ) + a31 * (a12 * bY - a22 * bX));
    return out;
}