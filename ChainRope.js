// ChainRope.js

var     world
    ,   positions           = [] // 과거 위치 배열
    ,   trailLength         = 100 // 잔상 길이
    ,   trailOpacity        = 0.2 // 잔상 투명도

var     b2Vec2              = Box2D.Common.Math.b2Vec2
    ,	b2BodyDef           = Box2D.Dynamics.b2BodyDef
    ,	b2Body              = Box2D.Dynamics.b2Body
    ,	b2FixtureDef        = Box2D.Dynamics.b2FixtureDef
    ,	b2Fixture           = Box2D.Dynamics.b2Fixture
    ,	b2World             = Box2D.Dynamics.b2World
    ,	b2MassData          = Box2D.Collision.Shapes.b2MassData
    ,	b2PolygonShape      = Box2D.Collision.Shapes.b2PolygonShape
    ,	b2CircleShape       = Box2D.Collision.Shapes.b2CircleShape
    ,   b2RevoluteJointDef  = Box2D.Dynamics.Joints.b2RevoluteJointDef
    ,   b2WeldJointDef      = Box2D.Dynamics.Joints.b2WeldJointDef
    ,	b2DebugDraw         = Box2D.Dynamics.b2DebugDraw
    ,   fixDef              = new b2FixtureDef
    ,   bodyDef             = new b2BodyDef


function init() {
    ctx.scale(128, 128)

    world = new b2World(
        new b2Vec2(0, 0.49)    // gravity
    ,   true                  // allow sleep
    )
    
    var groundWidth             = window.innerWidth
    ,   groundHeight            = window.innerHeight
        fixDef.density          = 1.0
        fixDef.friction         = 0.0
        fixDef.restitution      = 0.0
        fixDef.filter.maskBits  = 1
    
    // StaticBody
    bodyDef.type         = b2Body.b2_staticBody

        root             = createCircle(groundWidth / 256, groundHeight / 1024, 0.02, 0, 0, 0)

    // DynamicBody
    bodyDef.type         = b2Body.b2_dynamicBody
    
    const NUM_EDGES     = 150
    const edgeLengths   = new Array(NUM_EDGES).fill(0.02)
    const edges         = []
    const otherEndsX    = []
    const otherEndsY    = []
    const jointAnchors  = []
    
    // 첫 번째 edge 생성
    let initialX        = groundWidth / 256 + 0.02
    let initialY        = groundHeight / 1024
    edges.push(createPolygon(initialX, initialY, 4, 0.02, 0.01, 0, 0, 0))
    
    for (let i = 0; i < NUM_EDGES; i++) {
        const edge = edges[i]
        const edgeAngle = edge.GetAngle()
        const otherEndX = edge.GetPosition().x + edgeLengths[i] * Math.cos(edgeAngle)
        const otherEndY = edge.GetPosition().y + edgeLengths[i] * Math.sin(edgeAngle)
    
        otherEndsX.push(otherEndX)
        otherEndsY.push(otherEndY)
    
        if (i < NUM_EDGES - 1) { // 마지막 edge가 아닌 경우
            edges.push(createPolygon(otherEndX + 0.02, otherEndY, 4, 0.02, 0.01, 0, 0, 0))
        } else { // 마지막 edge인 경우
            end = createCircle(otherEndX, otherEndY, 0.02, 0, 0, 0)
        }
    
        jointAnchors.push(new b2Vec2(otherEndX, otherEndY))
    }
    
    // joint 생성
    createRevoluteJoint(root, edges[0], root.GetWorldCenter())
    for (let i = 0; i < NUM_EDGES - 1; i++) {
        createRevoluteJoint(edges[i], edges[i + 1], jointAnchors[i])
    }
    createWeldJoint(edges[NUM_EDGES - 1], end, jointAnchors[NUM_EDGES - 1])
    window.setInterval(update, 1000 / 240)
}

function createCircle(x, y, size, vx, vy, av) {
    fixDef.shape = new b2CircleShape (size)
    bodyDef.position.x              = x
    bodyDef.position.y              = y
    var     initialVelocityX        = vx
    ,       initialVelocityY        = vy
    ,       initialAngularVelocity  = av
    var     objBody                 = world.CreateBody(bodyDef)
            objBody.CreateFixture(fixDef)
            objBody.SetLinearVelocity(new b2Vec2(initialVelocityX, initialVelocityY))
            objBody.SetAngularVelocity(initialAngularVelocity)
    return  objBody
}

function createPolygon(x, y, n, Width, Height, vx, vy, av) {
    var vertices = []
    ,   angle = (Math.PI * 2) / n
    for (let i = 0; i < n; i++) {
        var vertexX = Math.cos(angle * i) * Width
        ,   vertexY = Math.sin(angle * i) * Height
        vertices.push(new b2Vec2(vertexX, vertexY))
    }
    fixDef.shape = new b2PolygonShape()
    fixDef.shape.SetAsArray(vertices, n)
    bodyDef.position.x              = x
    bodyDef.position.y              = y
    var     initialVelocityX        = vx
    ,       initialVelocityY        = vy
    ,       initialAngularVelocity  = av
    var     objBody                 = world.CreateBody(bodyDef)
            objBody.CreateFixture(fixDef)
            objBody.SetLinearVelocity(new b2Vec2(initialVelocityX, initialVelocityY))
            objBody.SetAngularVelocity(initialAngularVelocity)
    return  objBody
}

function createWeldJoint(bodyA, bodyB, anchor) {
    var     jointDef = new b2WeldJointDef()
            jointDef.Initialize(bodyA, bodyB, anchor)
    return  world.CreateJoint(jointDef)
}

function createRevoluteJoint(bodyA, bodyB, anchor) {
    var     jointDef = new b2RevoluteJointDef()
            jointDef.Initialize(bodyA, bodyB, anchor)
    return  world.CreateJoint(jointDef)
}

function update() {
    world.Step(1 / 120, 10, 10)
    
    ctx.clearRect(0, 0, canvas.width, canvas.height)
    
    // Draw trail
    for (var i = positions.length - 1; i >= 0; i--) {
        var position = positions[i]
        ,   alpha = (trailOpacity / trailLength) * (i + 1) // 투명도 계산 (역순)
        
        ctx.save()
        ctx.translate(position.x, position.y)
        
        // Draw a circle
        ctx.beginPath()
        ctx.arc(0, 0, 0.02, 0, 2 * Math.PI)
        ctx.fillStyle = "rgba(255, 255, 255, " + alpha + ")"
        ctx.fill()
        
        ctx.restore()
    }
    
    // Update positions array
    positions.push({ x: end.GetPosition().x, y: end.GetPosition().y })
    if (positions.length > trailLength) {
        positions.shift()
    }
    
    // Draw all bodies
    for (var body = world.GetBodyList(); body; body = body.GetNext()) {
        var fixture = body.GetFixtureList()
        if (fixture) { // Only proceed if the body has a fixture
            var shape = fixture.GetShape()
            ,   type = shape.GetType()

            ctx.save()
            ctx.translate(body.GetPosition().x, body.GetPosition().y)
            ctx.rotate(body.GetAngle())
            
            if (type == Box2D.Collision.Shapes.b2Shape.e_circleShape) {
                // Draw a circle
                ctx.beginPath()
                ctx.arc(0, 0, shape.GetRadius(), 0, 2 * Math.PI)
                ctx.fillStyle = "#FFFFFF"
                ctx.fill()
            } else if (type == Box2D.Collision.Shapes.b2Shape.e_polygonShape) {
                // Draw a polygon
                var vertices = shape.GetVertices()
                ctx.beginPath()
                ctx.moveTo(vertices[0].x, vertices[0].y)
                for (var i = 1; i < vertices.length; i++) {
                    ctx.lineTo(vertices[i].x, vertices[i].y)
                }
                ctx.closePath()
                ctx.fillStyle = body.GetType() == Box2D.Dynamics.b2Body.b2_staticBody ? "transparent" : "#FFFFFF"
                ctx.fill()
            }
            
            ctx.restore()
        }
    }
    if (mouseBody) {
        mouseBody.SetPosition(new b2Vec2(mouseX, mouseY));
    }

    world.ClearForces()
}

//mouse interection
let mouseX = 0;
let mouseY = 0;

canvas.addEventListener('mousemove', function(e) {
    mouseX = e.clientX / 128; // 128은 init 함수에서의 ctx.scale 값
    mouseY = e.clientY / 128;
});

let mouseBody = null;

canvas.addEventListener('mousedown', function() {
    if (!mouseBody) {
        bodyDef.type         = b2Body.b2_staticBody
        mouseBody = createCircle(mouseX, mouseY, 0.1, 0, 0, 0); // 마우스 위치에 작은 원을 생성
    }
});

canvas.addEventListener('mouseup', function() {
    if (mouseBody) {
        world.DestroyBody(mouseBody);
        mouseBody = null;
    }
});
