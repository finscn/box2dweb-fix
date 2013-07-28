function b2ContactFactory(allocator) {
   this.m_allocator = allocator;
   this.InitializeRegisters();
}

Box2D.Dynamics.Contacts.b2ContactFactory = b2ContactFactory;

b2ContactFactory.prototype.AddType = function(createFcn, destroyFcn, type1, type2) {
   if (type1 === undefined) type1 = 0;
   if (type2 === undefined) type2 = 0;
   this.m_registers[type1][type2].createFcn = createFcn;
   this.m_registers[type1][type2].destroyFcn = destroyFcn;
   this.m_registers[type1][type2].primary = true;
   if (type1 != type2) {
      this.m_registers[type2][type1].createFcn = createFcn;
      this.m_registers[type2][type1].destroyFcn = destroyFcn;
      this.m_registers[type2][type1].primary = false;
   }
}
b2ContactFactory.prototype.InitializeRegisters = function() {
   this.m_registers = new Vector(b2Shape.e_shapeTypeCount);
   for (var i = 0; i < b2Shape.e_shapeTypeCount; i++) {
      this.m_registers[i] = new Vector(b2Shape.e_shapeTypeCount);
      for (var j = 0; j < b2Shape.e_shapeTypeCount; j++) {
         this.m_registers[i][j] = new b2ContactRegister();
      }
   }
   this.AddType(b2CircleContact.Create, b2CircleContact.Destroy, b2Shape.e_circleShape, b2Shape.e_circleShape);
   this.AddType(b2PolyAndCircleContact.Create, b2PolyAndCircleContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_circleShape);
   this.AddType(b2PolygonContact.Create, b2PolygonContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_polygonShape);
   this.AddType(b2EdgeAndCircleContact.Create, b2EdgeAndCircleContact.Destroy, b2Shape.e_edgeShape, b2Shape.e_circleShape);
   this.AddType(b2PolyAndEdgeContact.Create, b2PolyAndEdgeContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_edgeShape);
}
b2ContactFactory.prototype.Create = function(fixtureA, fixtureB) {
   var type1 = parseInt(fixtureA.GetType());
   var type2 = parseInt(fixtureB.GetType());
   var reg = this.m_registers[type1][type2];
   var c;
   if (reg.pool) {
      c = reg.pool;
      reg.pool = c.m_next;
      reg.poolCount--;
      c.Reset(fixtureA, fixtureB);
      return c;
   }
   var createFcn = reg.createFcn;
   if (createFcn != null) {
      if (reg.primary) {
         c = createFcn(this.m_allocator);
         c.Reset(fixtureA, fixtureB);
         return c;
      } else {
         c = createFcn(this.m_allocator);
         c.Reset(fixtureB, fixtureA);
         return c;
      }
   } else {
      return null;
   }
}
b2ContactFactory.prototype.Destroy = function(contact) {
   if (contact.m_manifold.m_pointCount > 0) {
      contact.m_fixtureA.m_body.SetAwake(true);
      contact.m_fixtureB.m_body.SetAwake(true);
   }
   var type1 = parseInt(contact.m_fixtureA.GetType());
   var type2 = parseInt(contact.m_fixtureB.GetType());
   var reg = this.m_registers[type1][type2];
   if (true) {
      reg.poolCount++;
      contact.m_next = reg.pool;
      reg.pool = contact;
   }
   var destroyFcn = reg.destroyFcn;
   destroyFcn(contact, this.m_allocator);
}