var fs = require('fs');

var inputDir="./lib/";
var outputFile="./dist/box2d-all-in-one.js";

var headFile="base.js";

var bodyFileList=[

"Common/b2Settings.js",
"Common/b2Mat22.js",
"Common/b2Mat33.js",
"Common/b2Math.js",
"Common/b2Sweep.js",
"Common/b2Transform.js",
"Common/b2Vec2.js",
"Common/b2Vec3.js",
"Common/b2Color.js",

"Collision/b2AABB.js",
"Collision/b2Bound.js",
"Collision/b2BoundValues.js",
"Collision/b2Collision.js",
"Collision/b2ContactID.js",
"Collision/b2ContactPoint.js",
"Collision/b2Distance.js",
"Collision/b2DistanceInput.js",
"Collision/b2DistanceOutput.js",
"Collision/b2DistanceProxy.js",
"Collision/b2DynamicTree.js",
"Collision/b2DynamicTreeBroadPhase.js",
"Collision/b2DynamicTreeNode.js",
"Collision/b2DynamicTreePair.js",
"Collision/b2Manifold.js",
"Collision/b2ManifoldPoint.js",
"Collision/b2Point.js",
"Collision/b2RayCastInput.js",
"Collision/b2RayCastOutput.js",
"Collision/b2Segment.js",
"Collision/b2SeparationFunction.js",
"Collision/b2Simplex.js",
"Collision/b2SimplexCache.js",
"Collision/b2SimplexVertex.js",
"Collision/b2TimeOfImpact.js",
"Collision/b2TOIInput.js",
"Collision/b2WorldManifold.js",

"Collision/Shape/b2Shape.js",
"Collision/Shape/b2CircleShape.js",
"Collision/Shape/b2EdgeChainDef.js",
"Collision/Shape/b2EdgeShape.js",
"Collision/Shape/b2MassData.js",
"Collision/Shape/b2PolygonShape.js",

"Dynamics/b2Body.js",
"Dynamics/b2BodyDef.js",
"Dynamics/b2ContactFilter.js",
"Dynamics/b2ContactImpulse.js",
"Dynamics/b2ContactListener.js",
"Dynamics/b2ContactManager.js",
"Dynamics/b2DebugDraw.js",
"Dynamics/b2DebugDraw-1.js",
"Dynamics/b2DestructionListener.js",
"Dynamics/b2FilterData.js",
"Dynamics/b2Fixture.js",
"Dynamics/b2FixtureDef.js",
"Dynamics/b2Island.js",
"Dynamics/b2TimeStep.js",
"Dynamics/b2World.js",

"Dynamics/Contact/b2Contact.js",
"Dynamics/Contact/b2CircleContact.js",
"Dynamics/Contact/b2ContactConstraint.js",
"Dynamics/Contact/b2ContactConstraintPoint.js",
"Dynamics/Contact/b2ContactEdge.js",
"Dynamics/Contact/b2ContactFactory.js",
"Dynamics/Contact/b2ContactRegister.js",
"Dynamics/Contact/b2ContactResult.js",
"Dynamics/Contact/b2ContactSolver.js",
"Dynamics/Contact/b2EdgeAndCircleContact.js",
"Dynamics/Contact/b2NullContact.js",
"Dynamics/Contact/b2PolyAndCircleContact.js",
"Dynamics/Contact/b2PolyAndEdgeContact.js",
"Dynamics/Contact/b2PolygonContact.js",
"Dynamics/Contact/b2PositionSolverManifold.js",

"Dynamics/Controller/b2Controller.js",
"Dynamics/Controller/b2BuoyancyController.js",
"Dynamics/Controller/b2ConstantAccelController.js",
"Dynamics/Controller/b2ConstantForceController.js",
"Dynamics/Controller/b2ControllerEdge.js",
"Dynamics/Controller/b2GravityController.js",
"Dynamics/Controller/b2TensorDampingController.js",

"Dynamics/Joints/b2Joint.js",
"Dynamics/Joints/b2JointDef.js",
"Dynamics/Joints/b2DistanceJoint.js",
"Dynamics/Joints/b2DistanceJointDef.js",
"Dynamics/Joints/b2FrictionJoint.js",
"Dynamics/Joints/b2FrictionJointDef.js",
"Dynamics/Joints/b2GearJoint.js",
"Dynamics/Joints/b2GearJointDef.js",
"Dynamics/Joints/b2Jacobian.js",
"Dynamics/Joints/b2JointEdge.js",
"Dynamics/Joints/b2LineJoint.js",
"Dynamics/Joints/b2LineJointDef.js",
"Dynamics/Joints/b2MouseJoint.js",
"Dynamics/Joints/b2MouseJointDef.js",
"Dynamics/Joints/b2PrismaticJoint.js",
"Dynamics/Joints/b2PrismaticJointDef.js",
"Dynamics/Joints/b2PulleyJoint.js",
"Dynamics/Joints/b2PulleyJointDef.js",
"Dynamics/Joints/b2RevoluteJoint.js",
"Dynamics/Joints/b2RevoluteJointDef.js",
"Dynamics/Joints/b2WeldJoint.js",
"Dynamics/Joints/b2WeldJointDef.js"
];
var tailFile="init.js";


var headCode=fs.readFileSync(inputDir+headFile,'utf8');
var tailCode=fs.readFileSync(inputDir+tailFile,'utf8');

var bodyCode, bodyCodes=[];
bodyFileList.forEach(function(bodyFile){
	bodyCode=fs.readFileSync(inputDir+bodyFile,'utf8');
	bodyCodes.push(bodyCode);
})
bodyCode=bodyCodes.join("\n\n");

var scopeStart="\n\n;(function(undefined){\n\n";
var scopeEnd="\n\n}());\n\n";
var allInOne=headCode + scopeStart + bodyCode + scopeEnd + tailCode;

fs.writeFileSync(outputFile,allInOne);



