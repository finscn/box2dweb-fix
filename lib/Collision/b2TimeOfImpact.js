function b2TimeOfImpact() {

};
Box2D.Collision.b2TimeOfImpact = b2TimeOfImpact;

b2TimeOfImpact.TimeOfImpact = function(input) {
   ++b2TimeOfImpact.b2_toiCalls;
   var proxyA = input.proxyA;
   var proxyB = input.proxyB;
   var sweepA = input.sweepA;
   var sweepB = input.sweepB;
   b2Settings.b2Assert(sweepA.t0 == sweepB.t0);
   b2Settings.b2Assert(1.0 - sweepA.t0 > Number.MIN_VALUE);
   var radius = proxyA.m_radius + proxyB.m_radius;
   var tolerance = input.tolerance;
   var alpha = 0.0;
   var k_maxIterations = 1000;
   var iter = 0;
   var target = 0.0;
   b2TimeOfImpact.s_cache.count = 0;
   var disInput=b2TimeOfImpact.s_distanceInput;
   disInput.useRadii = false;
   for (;;) {
      sweepA.GetTransform(b2TimeOfImpact.s_xfA, alpha);
      sweepB.GetTransform(b2TimeOfImpact.s_xfB, alpha);
      disInput.proxyA = proxyA;
      disInput.proxyB = proxyB;
      disInput.transformA = b2TimeOfImpact.s_xfA;
      disInput.transformB = b2TimeOfImpact.s_xfB;
      b2Distance.Distance(b2TimeOfImpact.s_distanceOutput, b2TimeOfImpact.s_cache, disInput);
      if (b2TimeOfImpact.s_distanceOutput.distance <= 0.0) {
         alpha = 1.0;
         break;
      }
      b2TimeOfImpact.s_fcn.Initialize(b2TimeOfImpact.s_cache, proxyA, b2TimeOfImpact.s_xfA, proxyB, b2TimeOfImpact.s_xfB);
      var separation = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB);
      if (separation <= 0.0) {
         alpha = 1.0;
         break;
      }
      if (iter == 0) {
         if (separation > radius) {
            target = b2Math.Max(radius - tolerance, 0.75 * radius);
         } else {
            target = b2Math.Max(separation - tolerance, 0.02 * radius);
         }
      }
      if (separation - target < 0.5 * tolerance) {
         if (iter == 0) {
            alpha = 1.0;
            break;
         }
         break;
      }
      var newAlpha = alpha; {
         var x1 = alpha;
         var x2 = 1.0;
         var f1 = separation;
         sweepA.GetTransform(b2TimeOfImpact.s_xfA, x2);
         sweepB.GetTransform(b2TimeOfImpact.s_xfB, x2);
         var f2 = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB);
         if (f2 >= target) {
            alpha = 1.0;
            break;
         }
         var rootIterCount = 0;
         for (;;) {
            var x = 0;
            if (rootIterCount & 1) {
               x = x1 + (target - f1) * (x2 - x1) / (f2 - f1);
            } else {
               x = 0.5 * (x1 + x2);
            }
            sweepA.GetTransform(b2TimeOfImpact.s_xfA, x);
            sweepB.GetTransform(b2TimeOfImpact.s_xfB, x);
            var f = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB);
            if (b2Math.Abs(f - target) < 0.025 * tolerance) {
               newAlpha = x;
               break;
            }
            if (f > target) {
               x1 = x;
               f1 = f;
            } else {
               x2 = x;
               f2 = f;
            }++rootIterCount;
            ++b2TimeOfImpact.b2_toiRootIters;
            if (rootIterCount == 50) {
               break;
            }
         }
         b2TimeOfImpact.b2_toiMaxRootIters = b2Math.Max(b2TimeOfImpact.b2_toiMaxRootIters, rootIterCount);
      }
      if (newAlpha < (1.0 + 100.0 * Number.MIN_VALUE) * alpha) {
         break;
      }
      alpha = newAlpha;
      iter++;
      ++b2TimeOfImpact.b2_toiIters;
      if (iter == k_maxIterations) {
         break;
      }
   }
   b2TimeOfImpact.b2_toiMaxIters = b2Math.Max(b2TimeOfImpact.b2_toiMaxIters, iter);
   return alpha;
}
Box2D.postDefs.push(function() {
   Box2D.Collision.b2TimeOfImpact.b2_toiCalls = 0;
   Box2D.Collision.b2TimeOfImpact.b2_toiIters = 0;
   Box2D.Collision.b2TimeOfImpact.b2_toiMaxIters = 0;
   Box2D.Collision.b2TimeOfImpact.b2_toiRootIters = 0;
   Box2D.Collision.b2TimeOfImpact.b2_toiMaxRootIters = 0;
   Box2D.Collision.b2TimeOfImpact.s_cache = new b2SimplexCache();
   Box2D.Collision.b2TimeOfImpact.s_distanceInput = new b2DistanceInput();
   Box2D.Collision.b2TimeOfImpact.s_xfA = new b2Transform();
   Box2D.Collision.b2TimeOfImpact.s_xfB = new b2Transform();
   Box2D.Collision.b2TimeOfImpact.s_fcn = new b2SeparationFunction();
   Box2D.Collision.b2TimeOfImpact.s_distanceOutput = new b2DistanceOutput();
});