function b2ContactID() {
    this.features = new Features();
    this.features._m_id = this;
};
Box2D.Collision.b2ContactID = b2ContactID;

b2ContactID.prototype.Set = function(id) {
    this.key = id._key;
}
b2ContactID.prototype.Copy = function() {
    var id = new b2ContactID();
    id.key = this.key;
    return id;
}
Object.defineProperty(b2ContactID.prototype, 'key', {
    enumerable: false,
    configurable: true,
    get: function() {
        return this._key;
    }
});
Object.defineProperty(b2ContactID.prototype, 'key', {
    enumerable: false,
    configurable: true,
    set: function(value) {
        if (value === undefined) value = 0;
        this._key = value;
        this.features._referenceEdge = this._key & 0x000000ff;
        this.features._incidentEdge = ((this._key & 0x0000ff00) >> 8) & 0x000000ff;
        this.features._incidentVertex = ((this._key & 0x00ff0000) >> 16) & 0x000000ff;
        this.features._flip = ((this._key & 0xff000000) >> 24) & 0x000000ff;
    }
});