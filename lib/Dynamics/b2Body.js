function b2Body(bd, world) {
   this.m_xf = new b2Transform();
   this.m_sweep = new b2Sweep();
   this.m_linearVelocity = new b2Vec2();
   this.m_force = new b2Vec2();

   this.m_flags = 0;
   if (bd.bullet) {
      this.m_flags |= b2Body.e_bulletFlag;
   }
   if (bd.fixedRotation) {
      this.m_flags |= b2Body.e_fixedRotationFlag;
   }
   if (bd.allowSleep) {
      this.m_flags |= b2Body.e_allowSleepFlag;
   }
   if (bd.awake) {
      this.m_flags |= b2Body.e_awakeFlag;
   }
   if (bd.active) {
      this.m_flags |= b2Body.e_activeFlag;
   }
   this.m_world = world;
   this.m_xf.position.SetV(bd.position);
   this.m_xf.R.Set(bd.angle);
   this.m_sweep.localCenter.SetZero();
   this.m_sweep.t0 = 1.0;
   this.m_sweep.a0 = this.m_sweep.a = bd.angle;
   var tMat = this.m_xf.R;
   var tVec = this.m_sweep.localCenter;
   this.m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
   this.m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
   this.m_sweep.c.x += this.m_xf.position.x;
   this.m_sweep.c.y += this.m_xf.position.y;
   this.m_sweep.c0.SetV(this.m_sweep.c);
   this.m_jointList = null;
   this.m_controllerList = null;
   this.m_contactList = null;
   this.m_controllerCount = 0;
   this.m_prev = null;
   this.m_next = null;
   this.m_linearVelocity.SetV(bd.linearVelocity);
   this.m_angularVelocity = bd.angularVelocity;
   this.m_linearDamping = bd.linearDamping;
   this.m_angularDamping = bd.angularDamping;
   this.m_force.Set(0.0, 0.0);
   this.m_torque = 0.0;
   this.m_sleepTime = 0.0;
   this.m_type = bd.type;
   if (this.m_type == b2Body.b2_dynamicBody) {
      this.m_mass = 1.0;
      this.m_invMass = 1.0;
   } else {
      this.m_mass = 0.0;
      this.m_invMass = 0.0;
   }
   this.m_I = 0.0;
   this.m_invI = 0.0;
   this.m_inertiaScale = bd.inertiaScale;
   this.m_userData = bd.userData;
   this.m_fixtureList = null;
   this.m_fixtureCount = 0;
};
Box2D.Dynamics.b2Body = b2Body;

b2Body.prototype.connectEdges = function(s1, s2, angle1) {
   if (angle1 === undefined) angle1 = 0;
   var angle2 = Math.atan2(s2.GetDirectionVector().y, s2.GetDirectionVector().x);
   var coreOffset = Math.tan((angle2 - angle1) * 0.5);
   var core = b2Math.MulFV(coreOffset, s2.GetDirectionVector());
   core = b2Math.SubtractVV(core, s2.GetNormalVector());
   core = b2Math.MulFV(b2Settings.b2_toiSlop, core);
   core = b2Math.AddVV(core, s2.GetVertex1());
   var cornerDir = b2Math.AddVV(s1.GetDirectionVector(), s2.GetDirectionVector());
   cornerDir.Normalize();
   var convex = b2Math.Dot(s1.GetDirectionVector(), s2.GetNormalVector()) > 0.0;
   s1.SetNextEdge(s2, core, cornerDir, convex);
   s2.SetPrevEdge(s1, core, cornerDir, convex);
   return angle2;
}
b2Body.prototype.CreateFixture = function(def) {
   if (this.m_world.IsLocked() == true) {
      return null;
   }
   var fixture = new b2Fixture();
   fixture.Create(this, this.m_xf, def);
   if (this.m_flags & b2Body.e_activeFlag) {
      var broadPhase = this.m_world.m_contactManager.m_broadPhase;
      fixture.CreateProxy(broadPhase, this.m_xf);
   }
   fixture.m_next = this.m_fixtureList;
   this.m_fixtureList = fixture;
   ++this.m_fixtureCount;
   fixture.m_body = this;
   if (fixture.m_density > 0.0) {
      this.ResetMassData();
   }
   this.m_world.m_flags |= b2World.e_newFixture;
   return fixture;
}
b2Body.prototype.CreateFixture2 = function(shape, density) {
   if (density === undefined) density = 0.0;
   var def = new b2FixtureDef();
   def.shape = shape;
   def.density = density;
   return this.CreateFixture(def);
}
b2Body.prototype.DestroyFixture = function(fixture) {
   if (this.m_world.IsLocked() == true) {
      return;
   }
   var node = this.m_fixtureList;
   var ppF = null;
   var found = false;
   while (node != null) {
      if (node == fixture) {
         if (ppF) ppF.m_next = fixture.m_next;
         else this.m_fixtureList = fixture.m_next;
         found = true;
         break;
      }
      ppF = node;
      node = node.m_next;
   }
   var edge = this.m_contactList;
   while (edge) {
      var c = edge.contact;
      edge = edge.next;
      var fixtureA = c.GetFixtureA();
      var fixtureB = c.GetFixtureB();
      if (fixture == fixtureA || fixture == fixtureB) {
         this.m_world.m_contactManager.Destroy(c);
      }
   }
   if (this.m_flags & b2Body.e_activeFlag) {
      var broadPhase = this.m_world.m_contactManager.m_broadPhase;
      fixture.DestroyProxy(broadPhase);
   } else {}
   fixture.Destroy();
   fixture.m_body = null;
   fixture.m_next = null;
   --this.m_fixtureCount;
   this.ResetMassData();
}
b2Body.prototype.SetPositionAndAngle = function(position, angle) {
   if (angle === undefined) angle = 0;
   var f;
   if (this.m_world.IsLocked() == true) {
      return;
   }
   this.m_xf.R.Set(angle);
   this.m_xf.position.SetV(position);
   var tMat = this.m_xf.R;
   var tVec = this.m_sweep.localCenter;
   this.m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
   this.m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
   this.m_sweep.c.x += this.m_xf.position.x;
   this.m_sweep.c.y += this.m_xf.position.y;
   this.m_sweep.c0.SetV(this.m_sweep.c);
   this.m_sweep.a0 = this.m_sweep.a = angle;
   var broadPhase = this.m_world.m_contactManager.m_broadPhase;
   for (f = this.m_fixtureList;
   f; f = f.m_next) {
      f.Synchronize(broadPhase, this.m_xf, this.m_xf);
   }
   this.m_world.m_contactManager.FindNewContacts();
}
b2Body.prototype.SetTransform = function(xf) {
   this.SetPositionAndAngle(xf.position, xf.GetAngle());
}
b2Body.prototype.GetTransform = function() {
   return this.m_xf;
}
b2Body.prototype.GetPosition = function() {
   return this.m_xf.position;
}
b2Body.prototype.SetPosition = function(position) {
   this.SetPositionAndAngle(position, this.GetAngle());
}
b2Body.prototype.GetAngle = function() {
   return this.m_sweep.a;
}
b2Body.prototype.SetAngle = function(angle) {
   if (angle === undefined) angle = 0;
   this.SetPositionAndAngle(this.GetPosition(), angle);
}
b2Body.prototype.GetWorldCenter = function() {
   return this.m_sweep.c;
}
b2Body.prototype.GetLocalCenter = function() {
   return this.m_sweep.localCenter;
}
b2Body.prototype.SetLinearVelocity = function(v) {
   if (this.m_type == b2Body.b2_staticBody) {
      return;
   }
   this.m_linearVelocity.SetV(v);
}
b2Body.prototype.GetLinearVelocity = function() {
   return this.m_linearVelocity;
}
b2Body.prototype.SetAngularVelocity = function(omega) {
   if (omega === undefined) omega = 0;
   if (this.m_type == b2Body.b2_staticBody) {
      return;
   }
   this.m_angularVelocity = omega;
}
b2Body.prototype.GetAngularVelocity = function() {
   return this.m_angularVelocity;
}
b2Body.prototype.GetDefinition = function() {
   var bd = new b2BodyDef();
   bd.type = this.GetType();
   bd.allowSleep = (this.m_flags & b2Body.e_allowSleepFlag) == b2Body.e_allowSleepFlag;
   bd.angle = this.GetAngle();
   bd.angularDamping = this.m_angularDamping;
   bd.angularVelocity = this.m_angularVelocity;
   bd.fixedRotation = (this.m_flags & b2Body.e_fixedRotationFlag) == b2Body.e_fixedRotationFlag;
   bd.bullet = (this.m_flags & b2Body.e_bulletFlag) == b2Body.e_bulletFlag;
   bd.awake = (this.m_flags & b2Body.e_awakeFlag) == b2Body.e_awakeFlag;
   bd.linearDamping = this.m_linearDamping;
   bd.linearVelocity.SetV(this.GetLinearVelocity());
   bd.position = this.GetPosition();
   bd.userData = this.GetUserData();
   return bd;
}
b2Body.prototype.ApplyForce = function(force, point) {
   if (this.m_type != b2Body.b2_dynamicBody) {
      return;
   }
   if (this.IsAwake() == false) {
      this.SetAwake(true);
   }
   this.m_force.x += force.x;
   this.m_force.y += force.y;
   this.m_torque += ((point.x - this.m_sweep.c.x) * force.y - (point.y - this.m_sweep.c.y) * force.x);
}
b2Body.prototype.ApplyTorque = function(torque) {
   if (torque === undefined) torque = 0;
   if (this.m_type != b2Body.b2_dynamicBody) {
      return;
   }
   if (this.IsAwake() == false) {
      this.SetAwake(true);
   }
   this.m_torque += torque;
}
b2Body.prototype.ApplyImpulse = function(impulse, point) {
   if (this.m_type != b2Body.b2_dynamicBody) {
      return;
   }
   if (this.IsAwake() == false) {
      this.SetAwake(true);
   }
   this.m_linearVelocity.x += this.m_invMass * impulse.x;
   this.m_linearVelocity.y += this.m_invMass * impulse.y;
   this.m_angularVelocity += this.m_invI * ((point.x - this.m_sweep.c.x) * impulse.y - (point.y - this.m_sweep.c.y) * impulse.x);
}
b2Body.prototype.Split = function(callback) {
   var linearVelocity = this.GetLinearVelocity().Copy();
   var angularVelocity = this.GetAngularVelocity();
   var center = this.GetWorldCenter();
   var body1 = this;
   var body2 = this.m_world.CreateBody(this.GetDefinition());
   var prev;
   for (var f = body1.m_fixtureList; f;) {
      if (callback(f)) {
         var next = f.m_next;
         if (prev) {
            prev.m_next = next;
         } else {
            body1.m_fixtureList = next;
         }
         body1.m_fixtureCount--;
         f.m_next = body2.m_fixtureList;
         body2.m_fixtureList = f;
         body2.m_fixtureCount++;
         f.m_body = body2;
         f = next;
      } else {
         prev = f;
         f = f.m_next;
      }
   }
   body1.ResetMassData();
   body2.ResetMassData();
   var center1 = body1.GetWorldCenter();
   var center2 = body2.GetWorldCenter();
   var velocity1 = b2Math.AddVV(linearVelocity, b2Math.CrossFV(angularVelocity, b2Math.SubtractVV(center1, center)));
   var velocity2 = b2Math.AddVV(linearVelocity, b2Math.CrossFV(angularVelocity, b2Math.SubtractVV(center2, center)));
   body1.SetLinearVelocity(velocity1);
   body2.SetLinearVelocity(velocity2);
   body1.SetAngularVelocity(angularVelocity);
   body2.SetAngularVelocity(angularVelocity);
   body1.SynchronizeFixtures();
   body2.SynchronizeFixtures();
   return body2;
}
b2Body.prototype.Merge = function(other) {
   var f;
   for (f = other.m_fixtureList;
   f;) {
      var next = f.m_next;
      other.m_fixtureCount--;
      f.m_next = this.m_fixtureList;
      this.m_fixtureList = f;
      this.m_fixtureCount++;
      f.m_body = body2;
      f = next;
   }
   body1.m_fixtureCount = 0;
   var body1 = this;
   var body2 = other;
   var center1 = body1.GetWorldCenter();
   var center2 = body2.GetWorldCenter();
   var velocity1 = body1.GetLinearVelocity().Copy();
   var velocity2 = body2.GetLinearVelocity().Copy();
   var angular1 = body1.GetAngularVelocity();
   var angular = body2.GetAngularVelocity();
   body1.ResetMassData();
   this.SynchronizeFixtures();
}
b2Body.prototype.GetMass = function() {
   return this.m_mass;
}
b2Body.prototype.GetInertia = function() {
   return this.m_I;
}
b2Body.prototype.GetMassData = function(data) {
   data.mass = this.m_mass;
   data.I = this.m_I;
   data.center.SetV(this.m_sweep.localCenter);
}
b2Body.prototype.SetMassData = function(massData) {
   b2Settings.b2Assert(this.m_world.IsLocked() == false);
   if (this.m_world.IsLocked() == true) {
      return;
   }
   if (this.m_type != b2Body.b2_dynamicBody) {
      return;
   }
   this.m_invMass = 0.0;
   this.m_I = 0.0;
   this.m_invI = 0.0;
   this.m_mass = massData.mass;
   if (this.m_mass <= 0.0) {
      this.m_mass = 1.0;
   }
   this.m_invMass = 1.0 / this.m_mass;
   if (massData.I > 0.0 && (this.m_flags & b2Body.e_fixedRotationFlag) == 0) {
      this.m_I = massData.I - this.m_mass * (massData.center.x * massData.center.x + massData.center.y * massData.center.y);
      this.m_invI = 1.0 / this.m_I;
   }
   var oldCenter = this.m_sweep.c.Copy();
   this.m_sweep.localCenter.SetV(massData.center);
   this.m_sweep.c0.SetV(b2Math.MulX(this.m_xf, this.m_sweep.localCenter));
   this.m_sweep.c.SetV(this.m_sweep.c0);
   this.m_linearVelocity.x += this.m_angularVelocity * (-(this.m_sweep.c.y - oldCenter.y));
   this.m_linearVelocity.y += this.m_angularVelocity * (+(this.m_sweep.c.x - oldCenter.x));
}
b2Body.prototype.ResetMassData = function() {
   this.m_mass = 0.0;
   this.m_invMass = 0.0;
   this.m_I = 0.0;
   this.m_invI = 0.0;
   this.m_sweep.localCenter.SetZero();
   if (this.m_type == b2Body.b2_staticBody || this.m_type == b2Body.b2_kinematicBody) {
      return;
   }
   var center = b2Vec2.Make(0, 0);
   for (var f = this.m_fixtureList; f; f = f.m_next) {
      if (f.m_density == 0.0) {
         continue;
      }
      var massData = f.GetMassData();
      this.m_mass += massData.mass;
      center.x += massData.center.x * massData.mass;
      center.y += massData.center.y * massData.mass;
      this.m_I += massData.I;
   }
   if (this.m_mass > 0.0) {
      this.m_invMass = 1.0 / this.m_mass;
      center.x *= this.m_invMass;
      center.y *= this.m_invMass;
   } else {
      this.m_mass = 1.0;
      this.m_invMass = 1.0;
   }
   if (this.m_I > 0.0 && (this.m_flags & b2Body.e_fixedRotationFlag) == 0) {
      this.m_I -= this.m_mass * (center.x * center.x + center.y * center.y);
      this.m_I *= this.m_inertiaScale;
      b2Settings.b2Assert(this.m_I > 0);
      this.m_invI = 1.0 / this.m_I;
   } else {
      this.m_I = 0.0;
      this.m_invI = 0.0;
   }
   var oldCenter = this.m_sweep.c.Copy();
   this.m_sweep.localCenter.SetV(center);
   this.m_sweep.c0.SetV(b2Math.MulX(this.m_xf, this.m_sweep.localCenter));
   this.m_sweep.c.SetV(this.m_sweep.c0);
   this.m_linearVelocity.x += this.m_angularVelocity * (-(this.m_sweep.c.y - oldCenter.y));
   this.m_linearVelocity.y += this.m_angularVelocity * (+(this.m_sweep.c.x - oldCenter.x));
}
b2Body.prototype.GetWorldPoint = function(localPoint) {
   var A = this.m_xf.R;
   var u = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y);
   u.x += this.m_xf.position.x;
   u.y += this.m_xf.position.y;
   return u;
}
b2Body.prototype.GetWorldVector = function(localVector) {
   return b2Math.MulMV(this.m_xf.R, localVector);
}
b2Body.prototype.GetLocalPoint = function(worldPoint) {
   return b2Math.MulXT(this.m_xf, worldPoint);
}
b2Body.prototype.GetLocalVector = function(worldVector) {
   return b2Math.MulTMV(this.m_xf.R, worldVector);
}
b2Body.prototype.GetLinearVelocityFromWorldPoint = function(worldPoint) {
   return new b2Vec2(this.m_linearVelocity.x - this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y), this.m_linearVelocity.y + this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x));
}
b2Body.prototype.GetLinearVelocityFromLocalPoint = function(localPoint) {
   var A = this.m_xf.R;
   var worldPoint = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y);
   worldPoint.x += this.m_xf.position.x;
   worldPoint.y += this.m_xf.position.y;
   return new b2Vec2(this.m_linearVelocity.x - this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y), this.m_linearVelocity.y + this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x));
}
b2Body.prototype.GetLinearDamping = function() {
   return this.m_linearDamping;
}
b2Body.prototype.SetLinearDamping = function(linearDamping) {
   if (linearDamping === undefined) linearDamping = 0;
   this.m_linearDamping = linearDamping;
}
b2Body.prototype.GetAngularDamping = function() {
   return this.m_angularDamping;
}
b2Body.prototype.SetAngularDamping = function(angularDamping) {
   if (angularDamping === undefined) angularDamping = 0;
   this.m_angularDamping = angularDamping;
}
b2Body.prototype.SetType = function(type) {
   if (type === undefined) type = 0;
   if (this.m_type == type) {
      return;
   }
   this.m_type = type;
   this.ResetMassData();
   if (this.m_type == b2Body.b2_staticBody) {
      this.m_linearVelocity.SetZero();
      this.m_angularVelocity = 0.0;
   }
   this.SetAwake(true);
   this.m_force.SetZero();
   this.m_torque = 0.0;
   for (var ce = this.m_contactList; ce; ce = ce.next) {
      ce.contact.FlagForFiltering();
   }
}
b2Body.prototype.GetType = function() {
   return this.m_type;
}
b2Body.prototype.SetBullet = function(flag) {
   if (flag) {
      this.m_flags |= b2Body.e_bulletFlag;
   } else {
      this.m_flags &= ~b2Body.e_bulletFlag;
   }
}
b2Body.prototype.IsBullet = function() {
   return (this.m_flags & b2Body.e_bulletFlag) == b2Body.e_bulletFlag;
}
b2Body.prototype.SetSleepingAllowed = function(flag) {
   if (flag) {
      this.m_flags |= b2Body.e_allowSleepFlag;
   } else {
      this.m_flags &= ~b2Body.e_allowSleepFlag;
      this.SetAwake(true);
   }
}
b2Body.prototype.SetAwake = function(flag) {
   if (flag) {
      this.m_flags |= b2Body.e_awakeFlag;
      this.m_sleepTime = 0.0;
   } else {
      this.m_flags&= ~b2Body.e_awakeFlag;
this.m_sleepTime = 0.0;
this.m_linearVelocity.SetZero();
this.m_angularVelocity = 0.0;
this.m_force.SetZero();
this.m_torque = 0.0;
}
}
b2Body.prototype.IsAwake = function() {
   return (this.m_flags & b2Body.e_awakeFlag) == b2Body.e_awakeFlag;
}
b2Body.prototype.SetFixedRotation = function(fixed) {
   if (fixed) {
      this.m_flags |= b2Body.e_fixedRotationFlag;
   } else {
      this.m_flags &= ~b2Body.e_fixedRotationFlag;
   }
   this.ResetMassData();
}
b2Body.prototype.IsFixedRotation = function() {
   return (this.m_flags & b2Body.e_fixedRotationFlag) == b2Body.e_fixedRotationFlag;
}
b2Body.prototype.SetActive = function(flag) {
   if (flag == this.IsActive()) {
      return;
   }
   var broadPhase;
   var f;
   if (flag) {
      this.m_flags |= b2Body.e_activeFlag;
      broadPhase = this.m_world.m_contactManager.m_broadPhase;
      for (f = this.m_fixtureList;
      f; f = f.m_next) {
         f.CreateProxy(broadPhase, this.m_xf);
      }
   } else {
      this.m_flags &= ~b2Body.e_activeFlag;
      broadPhase = this.m_world.m_contactManager.m_broadPhase;
      for (f = this.m_fixtureList;
      f; f = f.m_next) {
         f.DestroyProxy(broadPhase);
      }
      var ce = this.m_contactList;
      while (ce) {
         var ce0 = ce;
         ce = ce.next;
         this.m_world.m_contactManager.Destroy(ce0.contact);
      }
      this.m_contactList = null;
   }
}
b2Body.prototype.IsActive = function() {
   return (this.m_flags & b2Body.e_activeFlag) == b2Body.e_activeFlag;
}
b2Body.prototype.IsSleepingAllowed = function() {
   return (this.m_flags & b2Body.e_allowSleepFlag) == b2Body.e_allowSleepFlag;
}
b2Body.prototype.GetFixtureList = function() {
   return this.m_fixtureList;
}
b2Body.prototype.GetJointList = function() {
   return this.m_jointList;
}
b2Body.prototype.GetControllerList = function() {
   return this.m_controllerList;
}
b2Body.prototype.GetContactList = function() {
   return this.m_contactList;
}
b2Body.prototype.GetNext = function() {
   return this.m_next;
}
b2Body.prototype.GetUserData = function() {
   return this.m_userData;
}
b2Body.prototype.SetUserData = function(data) {
   this.m_userData = data;
}
b2Body.prototype.GetWorld = function() {
   return this.m_world;
}

b2Body.prototype.SynchronizeFixtures = function() {
   var xf1 = b2Body.s_xf1;
   xf1.R.Set(this.m_sweep.a0);
   var tMat = xf1.R;
   var tVec = this.m_sweep.localCenter;
   xf1.position.x = this.m_sweep.c0.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
   xf1.position.y = this.m_sweep.c0.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
   var f;
   var broadPhase = this.m_world.m_contactManager.m_broadPhase;
   for (f = this.m_fixtureList;
   f; f = f.m_next) {
      f.Synchronize(broadPhase, xf1, this.m_xf);
   }
}
b2Body.prototype.SynchronizeTransform = function() {
   this.m_xf.R.Set(this.m_sweep.a);
   var tMat = this.m_xf.R;
   var tVec = this.m_sweep.localCenter;
   this.m_xf.position.x = this.m_sweep.c.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
   this.m_xf.position.y = this.m_sweep.c.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
}
b2Body.prototype.ShouldCollide = function(other) {
   if (this.m_type != b2Body.b2_dynamicBody && other.m_type != b2Body.b2_dynamicBody) {
      return false;
   }
   for (var jn = this.m_jointList; jn; jn = jn.next) {
      if (jn.other == other) if (jn.joint.m_collideConnected == false) {
         return false;
      }
   }
   return true;
}
b2Body.prototype.Advance = function(t) {
   if (t === undefined) t = 0;
   this.m_sweep.Advance(t);
   this.m_sweep.c.SetV(this.m_sweep.c0);
   this.m_sweep.a = this.m_sweep.a0;
   this.SynchronizeTransform();
}
Box2D.postDefs.push(function() {
   Box2D.Dynamics.b2Body.s_xf1 = new b2Transform();
   Box2D.Dynamics.b2Body.e_islandFlag = 0x0001;
   Box2D.Dynamics.b2Body.e_awakeFlag = 0x0002;
   Box2D.Dynamics.b2Body.e_allowSleepFlag = 0x0004;
   Box2D.Dynamics.b2Body.e_bulletFlag = 0x0008;
   Box2D.Dynamics.b2Body.e_fixedRotationFlag = 0x0010;
   Box2D.Dynamics.b2Body.e_activeFlag = 0x0020;
   Box2D.Dynamics.b2Body.b2_staticBody = 0;
   Box2D.Dynamics.b2Body.b2_kinematicBody = 1;
   Box2D.Dynamics.b2Body.b2_dynamicBody = 2;
});