function b2FilterData() {
    this.categoryBits = 0x0001;
    this.maskBits = 0xFFFF;
    this.groupIndex = 0;

};
Box2D.Dynamics.b2FilterData = b2FilterData;

b2FilterData.prototype.Copy = function() {
    var copy = new b2FilterData();
    copy.categoryBits = this.categoryBits;
    copy.maskBits = this.maskBits;
    copy.groupIndex = this.groupIndex;
    return copy;
}