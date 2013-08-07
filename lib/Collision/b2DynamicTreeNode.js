function b2DynamicTreeNode() {
    this.aabb = new b2AABB();
};
Box2D.Collision.b2DynamicTreeNode = b2DynamicTreeNode;

b2DynamicTreeNode.prototype.IsLeaf = function() {
    return this.child1 == null;
}