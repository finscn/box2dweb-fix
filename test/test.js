      var balls=[];
      function randomInt(lower, higher) {
         return ((higher - lower + 1) * Math.random() + lower)>>0;
      }

      function init() {
         var   b2Vec2 = Box2D.Common.Math.b2Vec2
            ,  b2AABB = Box2D.Collision.b2AABB
         	,	b2BodyDef = Box2D.Dynamics.b2BodyDef
         	,	b2Body = Box2D.Dynamics.b2Body
         	,	b2FixtureDef = Box2D.Dynamics.b2FixtureDef
         	,	b2Fixture = Box2D.Dynamics.b2Fixture
         	,	b2World = Box2D.Dynamics.b2World
         	,	b2MassData = Box2D.Collision.Shapes.b2MassData
         	,	b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
         	,	b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
         	,	b2DebugDraw = Box2D.Dynamics.b2DebugDraw
            ,  b2MouseJointDef = Box2D.Dynamics.Joints.b2MouseJointDef
            ,  b2WorldManifold = Box2D.Collision.b2WorldManifold

            ;
         
         var world = new b2World(
               new b2Vec2(0, 0)    //gravity
            ,  true                 //allow sleep
         );
         
         var fixDef = new b2FixtureDef;
         // fixDef.density = 1.0;
         // fixDef.friction = 0.5;
         // fixDef.restitution = 0.2;
         
         var bodyDef = new b2BodyDef;
         bodyDef.allowSleep=false;

         //create ground
         bodyDef.type = b2Body.b2_staticBody;

         fixDef.shape = new b2PolygonShape;
         
         // fixDef.shape.SetAsBox(20, 2);

         // // bodyDef.position.Set(10, -1.8);
         // // var body=world.CreateBody(bodyDef);
         // // body.CreateFixture(fixDef);

         // bodyDef.position.Set(20, 480/30+1);
         // var body=world.CreateBody(bodyDef);
         // body.CreateFixture(fixDef);


         // fixDef.shape.SetAsBox(2, 20);

         // bodyDef.position.Set(-1.8, 240/30);
         // var body=world.CreateBody(bodyDef);
         // body.CreateFixture(fixDef);

         // bodyDef.position.Set(800/30+1, 240/30);
         // var body=world.CreateBody(bodyDef);
         // body.CreateFixture(fixDef);
         
         
         //create some objects
         // fixDef.isSensor=true;

         fixDef.shape = new b2PolygonShape;
         bodyDef.type = b2Body.b2_dynamicBody;  


         fixDef.shape.SetAsBox(2, 2);
         // bodyDef.angularVelocity=0.5;
         bodyDef.angle=Math.PI/4+0.3;
         bodyDef.position.x=400/30+1
         bodyDef.position.y=10;
         // bodyDef.linearVelocity.y=1;
         var body=world.CreateBody(bodyDef);
         body.CreateFixture(fixDef)


         fixDef.shape.SetAsBox(3, 2);
         // bodyDef.angularVelocity=0.5;
         bodyDef.angle=Math.PI/4;
         // bodyDef.angle=0;
         bodyDef.position.x=400/30+1
         bodyDef.position.y=14
         var body=world.CreateBody(bodyDef);
         body.CreateFixture(fixDef)
         var m=body.m_fixtureList.GetMassData()
         // console.log(body.m_fixtureList.m_shape.ResetMass())
         body.m_fixtureList.m_shape.ComputeMass(m,1)
         console.log(m)



         // bodyDef.angle=0;
         // bodyDef.position.x=400/30+1
         // bodyDef.position.y=6
         // // bodyDef.linearVelocity.y=2;
         // var body=world.CreateBody(bodyDef);
         // body.CreateFixture(fixDef)

         // for (var i=0;i<3;i++){
         //    if (i<12){
         //       bodyDef.type = b2Body.b2_dynamicBody;
         //       bodyDef.linearDamping = 2;  
         //    }else{
         //       bodyDef.type = b2Body.b2_staticBody;  
         //       bodyDef.linearDamping = 0;  
         //    }
         //    bodyDef.position.x = Math.random()*(790/30>>0);
         //    bodyDef.position.y = Math.random()*(470/30>>0);
         //    var ball=var body=world.CreateBody(bodyDef);
         //    ball.CreateFixture(fixDef);
         //    balls.push(ball);
         // }


         // bodyDef.type = b2Body.b2_dynamicBody;
         // for(var i = 0; i < 10; ++i) {
         //    if(Math.random() > 0.5) {
         //       fixDef.shape = new b2PolygonShape;
         //       fixDef.shape.SetAsBox(
         //             Math.random() + 0.1 //half width
         //          ,  Math.random() + 0.1 //half height
         //       );
         //    } else {
         //       fixDef.shape = new b2CircleShape(
         //          Math.random() + 0.1 //radius
         //       );
         //    }
         //    bodyDef.position.x = Math.random() * 10;
         //    bodyDef.position.y = Math.random() * 10;
         //    var body=world.CreateBody(bodyDef);
         body.CreateFixture(fixDef);
         // }
         
         //setup debug draw

         var canvas=document.getElementById("canvas")
         var context=canvas.getContext("2d")
         context.fillStyle="#ff0000";
         var debugDraw = new b2DebugDraw();
			debugDraw.SetSprite(context);
			debugDraw.SetDrawScale(30.0);
			debugDraw.SetFillAlpha(0.4);
			debugDraw.SetLineThickness(0);
			debugDraw.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit 
            | b2DebugDraw.e_controllerBit | b2DebugDraw.e_pairBit);
          // | b2DebugDraw.e_aabbBit
			world.SetDebugDraw(debugDraw);
         
         window.setInterval(update, 100);
         
         //mouse
         
         var mouseX, mouseY, mousePVec, isMouseDown, selectedBody, mouseJoint;
         var canvasPosition = getElementPosition(document.getElementById("canvas"));
         

         //update
         
         function update() {
            
            worldManifoldList.length=0;



            if(isMouseDown && (!mouseJoint)) {
               var body = getBodyAtMouse();
               if(body) {
                  var md = new b2MouseJointDef();
                  md.bodyA = world.GetGroundBody();
                  md.bodyB = body;
                  md.target.Set(mouseX, mouseY);
                  md.collideConnected = true;
                  md.maxForce = 300.0 * body.GetMass();
                  mouseJoint = world.CreateJoint(md);
                  body.SetAwake(true);
               }
            }
            
            if(mouseJoint) {
               if(isMouseDown) {
                  mouseJoint.SetTarget(new b2Vec2(mouseX, mouseY));
               } else {
                  world.DestroyJoint(mouseJoint);
                  mouseJoint = null;
               }
            }
         
            world.Step(1 / 60, 100, 100);


            context.clearRect(0,0,canvas.width,canvas.height);
            // balls.forEach(function(b,idx){

            //    if (idx<12 && !b.IsAwake()){
            //       b.m_linearVelocity.x=randomInt(-99,99);
            //       b.m_linearVelocity.y=randomInt(-99,99);

            //       b.SetAwake(true);
            //    }
            //    // var x=b.m_xf.position.x;
            //    // var y=b.m_xf.position.y;
            //    // context.fillRect(x*30,y*30,20,20)               
            // })

            world.DrawDebugData();

          var c=world.m_contactManager.m_world.m_contactList;
while(c){
          var _worldManifold=new b2WorldManifold();
         c.GetWorldManifold(_worldManifold);
         worldManifoldList.push(_worldManifold);
      c=c.GetNext()
}  


            context.fillStyle="red"
            worldManifoldList.forEach(function(wm){
               var len= wm.m_points.length;
               for (var i = 0; i < len; ++i) {
                 var mp = wm.m_points[i];

                 context.fillRect(mp.x*30,mp.y*30,4,4)   
               }
            })
            world.ClearForces();
         };
         
         //helpers
         
         //http://js-tut.aardon.de/js-tut/tutorial/position.html
         function getElementPosition(element) {
            var elem=element, tagname="", x=0, y=0;
           
            while((typeof(elem) == "object") && (typeof(elem.tagName) != "undefined")) {
               y += elem.offsetTop;
               x += elem.offsetLeft;
               tagname = elem.tagName.toUpperCase();

               if(tagname == "BODY")
                  elem=0;

               if(typeof(elem) == "object") {
                  if(typeof(elem.offsetParent) == "object")
                     elem = elem.offsetParent;
               }
            }

            return {x: x, y: y};
         }


      };

var worldManifoldList=[];