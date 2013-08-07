function b2BuoyancyController() {
   Box2D.Dynamics.Controllers.b2Controller.apply(this, arguments);
   this.normal = new b2Vec2(0, (-1));
   this.offset = 0;
   this.density = 0;
   this.velocity = new b2Vec2(0, 0);
   this.linearDrag = 2;
   this.angularDrag = 1;
   this.useDensity = false;
   this.useWorldGravity = true;
   this.gravity = null;

};
Box2D.Dynamics.Controllers.b2BuoyancyController = b2BuoyancyController;
Box2D.inherit(b2BuoyancyController, Box2D.Dynamics.Controllers.b2Controller);
b2BuoyancyController.prototype.__super = Box2D.Dynamics.Controllers.b2Controller.prototype;

b2BuoyancyController.prototype.Step = function(step) {
   if (!this.m_bodyList) return;
   if (this.useWorldGravity) {
      this.gravity = this.GetWorld().GetGravity().Copy();
   }
   for (var i = this.m_bodyList; i; i = i.nextBody) {
      var body = i.body;
      if (body.IsAwake() == false) {
         continue;
      }
      var areac = new b2Vec2();
      var massc = new b2Vec2();
      var area = 0.0;
      var mass = 0.0;
      for (var fixture = body.GetFixtureList(); fixture; fixture = fixture.GetNext()) {
         var sc = new b2Vec2();
         var sarea = fixture.GetShape().ComputeSubmergedArea(this.normal, this.offset, body.GetTransform(), sc);
         area += sarea;
         areac.x += sarea * sc.x;
         areac.y += sarea * sc.y;
         var shapeDensity = 0;
         if (this.useDensity) {
            shapeDensity = 1;
         } else {
            shapeDensity = 1;
         }
         mass += sarea * shapeDensity;
         massc.x += sarea * sc.x * shapeDensity;
         massc.y += sarea * sc.y * shapeDensity;
      }
      areac.x /= area;
      areac.y /= area;
      massc.x /= mass;
      massc.y /= mass;
      if (area < Number.MIN_VALUE) continue;
      var buoyancyForce = this.gravity.GetNegative();
      buoyancyForce.Multiply(this.density * area);
      body.ApplyForce(buoyancyForce, massc);
      var dragForce = body.GetLinearVelocityFromWorldPoint(areac);
      dragForce.Subtract(this.velocity);
      dragForce.Multiply((-this.linearDrag * area));
      body.ApplyForce(dragForce, areac);
      body.ApplyTorque((-body.GetInertia() / body.GetMass() * area * body.GetAngularVelocity() * this.angularDrag));
   }
}
b2BuoyancyController.prototype.Draw = function(debugDraw) {
   var r = 1000;
   var p1 = new b2Vec2();
   var p2 = new b2Vec2();
   p1.x = this.normal.x * this.offset + this.normal.y * r;
   p1.y = this.normal.y * this.offset - this.normal.x * r;
   p2.x = this.normal.x * this.offset - this.normal.y * r;
   p2.y = this.normal.y * this.offset + this.normal.x * r;
   var color = new b2Color(0, 0, 1);
   debugDraw.DrawSegment(p1, p2, color);
}