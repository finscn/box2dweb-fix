function b2DynamicTree() {
    this.m_root = null;
    this.m_freeList = null;
    this.m_path = 0;
    this.m_insertionCount = 0;
};
Box2D.Collision.b2DynamicTree = b2DynamicTree;

b2DynamicTree.prototype.CreateProxy = function(aabb, userData) {
    var node = this.AllocateNode();
    var extendX = b2Settings.b2_aabbExtension;
    var extendY = b2Settings.b2_aabbExtension;
    node.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
    node.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
    node.aabb.upperBound.x = aabb.upperBound.x + extendX;
    node.aabb.upperBound.y = aabb.upperBound.y + extendY;
    node.userData = userData;
    this.InsertLeaf(node);
    return node;
}
b2DynamicTree.prototype.DestroyProxy = function(proxy) {
    this.RemoveLeaf(proxy);
    this.FreeNode(proxy);
}
b2DynamicTree.prototype.MoveProxy = function(proxy, aabb, displacement) {
    b2Settings.b2Assert(proxy.IsLeaf());
    if (proxy.aabb.Contains(aabb)) {
        return false;
    }
    this.RemoveLeaf(proxy);
    var extendX = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * (displacement.x > 0 ? displacement.x : (-displacement.x));
    var extendY = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * (displacement.y > 0 ? displacement.y : (-displacement.y));
    proxy.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
    proxy.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
    proxy.aabb.upperBound.x = aabb.upperBound.x + extendX;
    proxy.aabb.upperBound.y = aabb.upperBound.y + extendY;
    this.InsertLeaf(proxy);
    return true;
}
b2DynamicTree.prototype.Rebalance = function(iterations) {
    if (iterations === undefined) iterations = 0;
    if (this.m_root == null) return;
    for (var i = 0; i < iterations; i++) {
        var node = this.m_root;
        var bit = 0;
        while (node.IsLeaf() == false) {
            node = (this.m_path >> bit) & 1 ? node.child2 : node.child1;
            bit = (bit + 1) & 31;
        }++this.m_path;
        this.RemoveLeaf(node);
        this.InsertLeaf(node);
    }
}
b2DynamicTree.prototype.GetFatAABB = function(proxy) {
    return proxy.aabb;
}
b2DynamicTree.prototype.GetUserData = function(proxy) {
    return proxy.userData;
}
b2DynamicTree.prototype.Query = function(callback, aabb) {
    if (this.m_root == null) return;
    var stack = new Vector();
    var count = 0;
    stack[count++] = this.m_root;
    while (count > 0) {
        var node = stack[--count];
        if (node.aabb.TestOverlap(aabb)) {
            if (node.IsLeaf()) {
                var proceed = callback(node);
                if (!proceed) return;
            } else {
                stack[count++] = node.child1;
                stack[count++] = node.child2;
            }
        }
    }
}
b2DynamicTree.prototype.RayCast = function(callback, input) {
    if (this.m_root == null) return;
    var p1 = input.p1;
    var p2 = input.p2;
    var r = b2Math.SubtractVV(p1, p2);
    r.Normalize();
    var v = b2Math.CrossFV(1.0, r);
    var abs_v = b2Math.AbsV(v);
    var maxFraction = input.maxFraction;
    var segmentAABB = new b2AABB();
    var tX = 0;
    var tY = 0; {
        tX = p1.x + maxFraction * (p2.x - p1.x);
        tY = p1.y + maxFraction * (p2.y - p1.y);
        segmentAABB.lowerBound.x = Math.min(p1.x, tX);
        segmentAABB.lowerBound.y = Math.min(p1.y, tY);
        segmentAABB.upperBound.x = Math.max(p1.x, tX);
        segmentAABB.upperBound.y = Math.max(p1.y, tY);
    }
    var stack = new Vector();
    var count = 0;
    stack[count++] = this.m_root;
    while (count > 0) {
        var node = stack[--count];
        if (node.aabb.TestOverlap(segmentAABB) == false) {
            continue;
        }
        var c = node.aabb.GetCenter();
        var h = node.aabb.GetExtents();
        var separation = Math.abs(v.x * (p1.x - c.x) + v.y * (p1.y - c.y)) - abs_v.x * h.x - abs_v.y * h.y;
        if (separation > 0.0) continue;
        if (node.IsLeaf()) {
            var subInput = new b2RayCastInput();
            subInput.p1 = input.p1;
            subInput.p2 = input.p2;
            subInput.maxFraction = input.maxFraction;
            maxFraction = callback(subInput, node);
            if (maxFraction == 0.0) return;
            if (maxFraction > 0.0) {
                tX = p1.x + maxFraction * (p2.x - p1.x);
                tY = p1.y + maxFraction * (p2.y - p1.y);
                segmentAABB.lowerBound.x = Math.min(p1.x, tX);
                segmentAABB.lowerBound.y = Math.min(p1.y, tY);
                segmentAABB.upperBound.x = Math.max(p1.x, tX);
                segmentAABB.upperBound.y = Math.max(p1.y, tY);
            }
        } else {
            stack[count++] = node.child1;
            stack[count++] = node.child2;
        }
    }
}
b2DynamicTree.prototype.AllocateNode = function() {
    if (this.m_freeList) {
        var node = this.m_freeList;
        this.m_freeList = node.parent;
        node.parent = null;
        node.child1 = null;
        node.child2 = null;
        return node;
    }
    return new b2DynamicTreeNode();
}
b2DynamicTree.prototype.FreeNode = function(node) {
    node.parent = this.m_freeList;
    this.m_freeList = node;
}
b2DynamicTree.prototype.InsertLeaf = function(leaf) {
    ++this.m_insertionCount;
    if (this.m_root == null) {
        this.m_root = leaf;
        this.m_root.parent = null;
        return;
    }
    var center = leaf.aabb.GetCenter();
    var sibling = this.m_root;
    if (sibling.IsLeaf() == false) {
        do {
            var child1 = sibling.child1;
            var child2 = sibling.child2;
            var norm1 = Math.abs((child1.aabb.lowerBound.x + child1.aabb.upperBound.x) / 2 - center.x) + Math.abs((child1.aabb.lowerBound.y + child1.aabb.upperBound.y) / 2 - center.y);
            var norm2 = Math.abs((child2.aabb.lowerBound.x + child2.aabb.upperBound.x) / 2 - center.x) + Math.abs((child2.aabb.lowerBound.y + child2.aabb.upperBound.y) / 2 - center.y);
            if (norm1 < norm2) {
                sibling = child1;
            } else {
                sibling = child2;
            }
        }
        while (sibling.IsLeaf() == false)
    }
    var node1 = sibling.parent;
    var node2 = this.AllocateNode();
    node2.parent = node1;
    node2.userData = null;
    node2.aabb.Combine(leaf.aabb, sibling.aabb);
    if (node1) {
        if (sibling.parent.child1 == sibling) {
            node1.child1 = node2;
        } else {
            node1.child2 = node2;
        }
        node2.child1 = sibling;
        node2.child2 = leaf;
        sibling.parent = node2;
        leaf.parent = node2;
        do {
            if (node1.aabb.Contains(node2.aabb)) break;
            node1.aabb.Combine(node1.child1.aabb, node1.child2.aabb);
            node2 = node1;
            node1 = node1.parent;
        }
        while (node1)
    } else {
        node2.child1 = sibling;
        node2.child2 = leaf;
        sibling.parent = node2;
        leaf.parent = node2;
        this.m_root = node2;
    }
}
b2DynamicTree.prototype.RemoveLeaf = function(leaf) {
    if (leaf == this.m_root) {
        this.m_root = null;
        return;
    }
    var node2 = leaf.parent;
    var node1 = node2.parent;
    var sibling;
    if (node2.child1 == leaf) {
        sibling = node2.child2;
    } else {
        sibling = node2.child1;
    }
    if (node1) {
        if (node1.child1 == node2) {
            node1.child1 = sibling;
        } else {
            node1.child2 = sibling;
        }
        sibling.parent = node1;
        this.FreeNode(node2);
        while (node1) {
            var oldAABB = node1.aabb;
            node1.aabb = b2AABB.Combine(node1.child1.aabb, node1.child2.aabb);
            if (oldAABB.Contains(node1.aabb)) break;
            node1 = node1.parent;
        }
    } else {
        this.m_root = sibling;
        sibling.parent = null;
        this.FreeNode(node2);
    }
}