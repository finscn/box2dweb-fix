function b2Vec2(x_, y_) {
    if (x_ === undefined) x_ = 0;
    if (y_ === undefined) y_ = 0;
    this.x = x_;
    this.y = y_;

};
Box2D.Common.Math.b2Vec2 = b2Vec2;

b2Vec2.prototype.SetZero = function() {
    this.x = 0.0;
    this.y = 0.0;
}
b2Vec2.prototype.Set = function(x_, y_) {
    if (x_ === undefined) x_ = 0;
    if (y_ === undefined) y_ = 0;
    this.x = x_;
    this.y = y_;
}
b2Vec2.prototype.SetV = function(v) {
    this.x = v.x;
    this.y = v.y;
}
b2Vec2.prototype.GetNegative = function() {
    return new b2Vec2((-this.x), (-this.y));
}
b2Vec2.prototype.NegativeSelf = function() {
    this.x = (-this.x);
    this.y = (-this.y);
}
b2Vec2.Make = function(x_, y_) {
    if (x_ === undefined) x_ = 0;
    if (y_ === undefined) y_ = 0;
    return new b2Vec2(x_, y_);
}
b2Vec2.prototype.Copy = function() {
    return new b2Vec2(this.x, this.y);
}
b2Vec2.prototype.Add = function(v) {
    this.x += v.x;
    this.y += v.y;
}
b2Vec2.prototype.Subtract = function(v) {
    this.x -= v.x;
    this.y -= v.y;
}
b2Vec2.prototype.Multiply = function(a) {
    if (a === undefined) a = 0;
    this.x *= a;
    this.y *= a;
}
b2Vec2.prototype.MulM = function(A) {
    var tX = this.x;
    this.x = A.col1.x * tX + A.col2.x * this.y;
    this.y = A.col1.y * tX + A.col2.y * this.y;
}
b2Vec2.prototype.MulTM = function(A) {
    var tX = b2Math.Dot(this, A.col1);
    this.y = b2Math.Dot(this, A.col2);
    this.x = tX;
}
b2Vec2.prototype.CrossVF = function(s) {
    if (s === undefined) s = 0;
    var tX = this.x;
    this.x = s * this.y;
    this.y = (-s * tX);
}
b2Vec2.prototype.CrossFV = function(s) {
    if (s === undefined) s = 0;
    var tX = this.x;
    this.x = (-s * this.y);
    this.y = s * tX;
}
b2Vec2.prototype.MinV = function(b) {
    this.x = this.x < b.x ? this.x : b.x;
    this.y = this.y < b.y ? this.y : b.y;
}
b2Vec2.prototype.MaxV = function(b) {
    this.x = this.x > b.x ? this.x : b.x;
    this.y = this.y > b.y ? this.y : b.y;
}
b2Vec2.prototype.Abs = function() {
    if (this.x < 0) this.x = (-this.x);
    if (this.y < 0) this.y = (-this.y);
}
b2Vec2.prototype.Length = function() {
    return Math.sqrt(this.x * this.x + this.y * this.y);
}
b2Vec2.prototype.LengthSquared = function() {
    return (this.x * this.x + this.y * this.y);
}
b2Vec2.prototype.Normalize = function() {
    var length = Math.sqrt(this.x * this.x + this.y * this.y);
    if (length < Number.MIN_VALUE) {
        return 0.0;
    }
    var invLength = 1.0 / length;
    this.x *= invLength;
    this.y *= invLength;
    return length;
}
b2Vec2.prototype.IsValid = function() {
    return b2Math.IsValid(this.x) && b2Math.IsValid(this.y);
}