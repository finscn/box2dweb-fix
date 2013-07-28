function b2DynamicTreeBroadPhase() {
   this.m_tree = new b2DynamicTree();
   this.m_moveBuffer = new Vector();
   this.m_pairBuffer = new Vector();
   this.m_pairCount = 0;

};
Box2D.Collision.b2DynamicTreeBroadPhase = b2DynamicTreeBroadPhase;

b2DynamicTreeBroadPhase.prototype.CreateProxy = function(aabb, userData) {
   var proxy = this.m_tree.CreateProxy(aabb, userData);
   ++this.m_proxyCount;
   this.BufferMove(proxy);
   return proxy;
}
b2DynamicTreeBroadPhase.prototype.DestroyProxy = function(proxy) {
   this.UnBufferMove(proxy);
   --this.m_proxyCount;
   this.m_tree.DestroyProxy(proxy);
}
b2DynamicTreeBroadPhase.prototype.MoveProxy = function(proxy, aabb, displacement) {
   var buffer = this.m_tree.MoveProxy(proxy, aabb, displacement);
   if (buffer) {
      this.BufferMove(proxy);
   }
}
b2DynamicTreeBroadPhase.prototype.TestOverlap = function(proxyA, proxyB) {
   var aabbA = this.m_tree.GetFatAABB(proxyA);
   var aabbB = this.m_tree.GetFatAABB(proxyB);
   return aabbA.TestOverlap(aabbB);
}
b2DynamicTreeBroadPhase.prototype.GetUserData = function(proxy) {
   return this.m_tree.GetUserData(proxy);
}
b2DynamicTreeBroadPhase.prototype.GetFatAABB = function(proxy) {
   return this.m_tree.GetFatAABB(proxy);
}
b2DynamicTreeBroadPhase.prototype.GetProxyCount = function() {
   return this.m_proxyCount;
}
b2DynamicTreeBroadPhase.prototype.UpdatePairs = function(callback) {
   var __this = this;
   __this.m_pairCount = 0;
   var i = 0,
      queryProxy;
   for (i = 0;
   i < __this.m_moveBuffer.length; ++i) {
      queryProxy = __this.m_moveBuffer[i];

      function QueryCallback(proxy) {
         if (proxy == queryProxy) return true;
         if (__this.m_pairCount == __this.m_pairBuffer.length) {
            __this.m_pairBuffer[__this.m_pairCount] = new b2DynamicTreePair();
         }
         var pair = __this.m_pairBuffer[__this.m_pairCount];
         pair.proxyA = proxy < queryProxy ? proxy : queryProxy;
         pair.proxyB = proxy >= queryProxy ? proxy : queryProxy;
         ++__this.m_pairCount;
         return true;
      };
      var fatAABB = __this.m_tree.GetFatAABB(queryProxy);
      __this.m_tree.Query(QueryCallback, fatAABB);
   }
   __this.m_moveBuffer.length = 0;
   for (var i = 0; i < __this.m_pairCount;) {
      var primaryPair = __this.m_pairBuffer[i];
      var userDataA = __this.m_tree.GetUserData(primaryPair.proxyA);
      var userDataB = __this.m_tree.GetUserData(primaryPair.proxyB);
      callback(userDataA, userDataB);
      ++i;
      while (i < __this.m_pairCount) {
         var pair = __this.m_pairBuffer[i];
         if (pair.proxyA != primaryPair.proxyA || pair.proxyB != primaryPair.proxyB) {
            break;
         }++i;
      }
   }
}
b2DynamicTreeBroadPhase.prototype.Query = function(callback, aabb) {
   this.m_tree.Query(callback, aabb);
}
b2DynamicTreeBroadPhase.prototype.RayCast = function(callback, input) {
   this.m_tree.RayCast(callback, input);
}
b2DynamicTreeBroadPhase.prototype.Validate = function() {}
b2DynamicTreeBroadPhase.prototype.Rebalance = function(iterations) {
   if (iterations === undefined) iterations = 0;
   this.m_tree.Rebalance(iterations);
}
b2DynamicTreeBroadPhase.prototype.BufferMove = function(proxy) {
   this.m_moveBuffer[this.m_moveBuffer.length] = proxy;
}
b2DynamicTreeBroadPhase.prototype.UnBufferMove = function(proxy) {
   var i = parseInt(this.m_moveBuffer.indexOf(proxy));
   this.m_moveBuffer.splice(i, 1);
}
b2DynamicTreeBroadPhase.prototype.ComparePairs = function(pair1, pair2) {
   return 0;
}
b2DynamicTreeBroadPhase.__implements = {};
b2DynamicTreeBroadPhase.__implements[IBroadPhase] = true;