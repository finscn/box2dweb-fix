function b2DebugDraw() {

};
Box2D.Dynamics.b2DebugDraw = b2DebugDraw;

b2DebugDraw.prototype.SetFlags = function(flags) {
    if (flags === undefined) flags = 0;
}
b2DebugDraw.prototype.GetFlags = function() {}
b2DebugDraw.prototype.AppendFlags = function(flags) {
    if (flags === undefined) flags = 0;
}
b2DebugDraw.prototype.ClearFlags = function(flags) {
    if (flags === undefined) flags = 0;
}
b2DebugDraw.prototype.SetSprite = function(sprite) {}
b2DebugDraw.prototype.GetSprite = function() {}
b2DebugDraw.prototype.SetDrawScale = function(drawScale) {
    if (drawScale === undefined) drawScale = 0;
}
b2DebugDraw.prototype.GetDrawScale = function() {}
b2DebugDraw.prototype.SetLineThickness = function(lineThickness) {
    if (lineThickness === undefined) lineThickness = 0;
}
b2DebugDraw.prototype.GetLineThickness = function() {}
b2DebugDraw.prototype.SetAlpha = function(alpha) {
    if (alpha === undefined) alpha = 0;
}
b2DebugDraw.prototype.GetAlpha = function() {}
b2DebugDraw.prototype.SetFillAlpha = function(alpha) {
    if (alpha === undefined) alpha = 0;
}
b2DebugDraw.prototype.GetFillAlpha = function() {}
b2DebugDraw.prototype.SetXFormScale = function(xformScale) {
    if (xformScale === undefined) xformScale = 0;
}
b2DebugDraw.prototype.GetXFormScale = function() {}
b2DebugDraw.prototype.DrawPolygon = function(vertices, vertexCount, color) {
    if (vertexCount === undefined) vertexCount = 0;
}
b2DebugDraw.prototype.DrawSolidPolygon = function(vertices, vertexCount, color) {
    if (vertexCount === undefined) vertexCount = 0;
}
b2DebugDraw.prototype.DrawCircle = function(center, radius, color) {
    if (radius === undefined) radius = 0;
}
b2DebugDraw.prototype.DrawSolidCircle = function(center, radius, axis, color) {
    if (radius === undefined) radius = 0;
}
b2DebugDraw.prototype.DrawSegment = function(p1, p2, color) {}
b2DebugDraw.prototype.DrawTransform = function(xf) {}
Box2D.postDefs.push(function() {
    Box2D.Dynamics.b2DebugDraw.e_shapeBit = 0x0001;
    Box2D.Dynamics.b2DebugDraw.e_jointBit = 0x0002;
    Box2D.Dynamics.b2DebugDraw.e_aabbBit = 0x0004;
    Box2D.Dynamics.b2DebugDraw.e_pairBit = 0x0008;
    Box2D.Dynamics.b2DebugDraw.e_centerOfMassBit = 0x0010;
    Box2D.Dynamics.b2DebugDraw.e_controllerBit = 0x0020;
});