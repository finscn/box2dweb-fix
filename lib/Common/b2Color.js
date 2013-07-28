function b2Color(rr, gg, bb) {
   this._r = 0;
   this._g = 0;
   this._b = 0;
   if (rr === undefined) rr = 0;
   if (gg === undefined) gg = 0;
   if (bb === undefined) bb = 0;
   this._r = Box2D.parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0));
   this._g = Box2D.parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0));
   this._b = Box2D.parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0));

};
Box2D.Common.b2Color = b2Color;

b2Color.prototype.Set = function(rr, gg, bb) {
   if (rr === undefined) rr = 0;
   if (gg === undefined) gg = 0;
   if (bb === undefined) bb = 0;
   this._r = Box2D.parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0));
   this._g = Box2D.parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0));
   this._b = Box2D.parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0));
}
Object.defineProperty(b2Color.prototype, 'r', {
   enumerable: false,
   configurable: true,
   set: function(rr) {
      if (rr === undefined) rr = 0;
      this._r = Box2D.parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0));
   }
});
Object.defineProperty(b2Color.prototype, 'g', {
   enumerable: false,
   configurable: true,
   set: function(gg) {
      if (gg === undefined) gg = 0;
      this._g = Box2D.parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0));
   }
});
Object.defineProperty(b2Color.prototype, 'b', {
   enumerable: false,
   configurable: true,
   set: function(bb) {
      if (bb === undefined) bb = 0;
      this._b = Box2D.parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0));
   }
});
Object.defineProperty(b2Color.prototype, 'color', {
   enumerable: false,
   configurable: true,
   get: function() {
      return (this._r << 16) | (this._g << 8) | (this._b);
   }
});