function b2Bound() {

};
Box2D.Collision.b2Bound = b2Bound;

b2Bound.prototype.IsLower = function() {
    return (this.value & 1) == 0;
}
b2Bound.prototype.IsUpper = function() {
    return (this.value & 1) == 1;
}
b2Bound.prototype.Swap = function(b) {
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