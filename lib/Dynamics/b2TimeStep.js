function b2TimeStep() {

};
Box2D.Dynamics.b2TimeStep = b2TimeStep;

b2TimeStep.prototype.Set = function(step) {
   this.dt = step.dt;
   this.inv_dt = step.inv_dt;
   this.positionIterations = step.positionIterations;
   this.velocityIterations = step.velocityIterations;
   this.warmStarting = step.warmStarting;
}