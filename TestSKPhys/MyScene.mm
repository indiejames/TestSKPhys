//
//  MyScene.m
//  TestSKPhys
//
//  Created by James Norton on 9/17/13.
//  Copyright (c) 2013 James Norton. All rights reserved.
//

#import "MyScene.h"
#import <Box2D.h>

#define BOX2D
#define NUM_BOXES (400)
#define PIXELS_PER_METER (150)
#define SMALL_BOX_WIDTH (16)
#define BIG_BOX_WIDTH (32)
#define SMALL_BOX_HEIGHT (SMALL_BOX_WIDTH / 2)
#define BIG_BOX_HEIGHT (BIG_BOX_WIDTH / 2)
#define ANGULAR_VELOCITY (100)
#define ANGULAR_DAMPING (0)
#define LINEAR_DAMPING (0)
#define DENSITY (1.0)
#define FRICTION (0.1)
#define RESTITUTION (0.1)
#define GRAVITY (-1.5)
#define MOV_AVG_COUNT (10)
#define FIRST_SAMPLE_POINT (0.5)
#define SECOND_SAMPLE_POINT (10.0)

float boxWidth = SMALL_BOX_WIDTH;
float boxHeight = SMALL_BOX_HEIGHT;

@implementation MyScene {
    b2World *world;
    double accumulator;
    double timeStep;
    double prevTime;
    int frameCount;
    double frameTimeTotal;
    double frameRate[MOV_AVG_COUNT];
    float g;
    BOOL frameRateDisplayed;
}

-(id)initWithSize:(CGSize)size {    
    if (self = [super initWithSize:size]) {
        /* Setup your scene here */
        timeStep = 1.0 / 60.0;
        accumulator = 0;
        prevTime = -1;
        frameCount = 0;
        frameTimeTotal = 0;
        g = GRAVITY;
        frameRateDisplayed = NO;
        
#ifdef BOX2D
        NSLog(@"Box2D - %d boxes", NUM_BOXES);
        
        b2Vec2 gravity(0.0f, g);
        bool doSleep = true;
        world = new b2World(gravity);
        world->SetAllowSleeping(doSleep);
        b2Vec2 vs[5];
        
        vs[0].Set(0, 0.0f);
        
        vs[1].Set(size.width / PIXELS_PER_METER, 0);
        
        vs[2].Set(size.width / PIXELS_PER_METER, size.height / PIXELS_PER_METER);
        
        vs[3].Set(0, size.height / PIXELS_PER_METER);
        
        vs[4].Set(0, 0);
        
        
        
        b2ChainShape chain;
        
        chain.CreateChain(vs, 5);
        
        b2BodyDef boundaryDef;
        boundaryDef.position.Set(0,0);
        b2Body *boundary = world->CreateBody(&boundaryDef);
        boundary->CreateFixture(&chain, 0);
#else
        NSLog(@"SKPhysics - %d boxes", NUM_BOXES);
        self.physicsBody = [SKPhysicsBody bodyWithEdgeLoopFromRect:self.frame];
        self.physicsWorld.gravity = CGVectorMake(0, g);
#endif
        self.backgroundColor = [SKColor colorWithRed:0.15 green:0.15 blue:0.3 alpha:1.0];
        
        [self addBoxes];
        
    }
    return self;
}

-(void)addBoxes {
    float xStart = boxWidth - 3;
    float yStart = self.size.height-2*boxHeight;
    
    int rowSize = self.size.width / (boxWidth + 5);
    
    for(int i = 0;i<NUM_BOXES;i++) {
        float x = (i % rowSize) * (boxWidth + 5) + xStart;
        float y = yStart - (i / rowSize) * (boxHeight * 1.5);
        [self addBoxAtX:x Y:y];
        
    }
}


- (void)addBoxAtX:(float )x Y:(float)y {
    
    
    
    
    SKSpriteNode *Box = [[SKSpriteNode alloc] initWithColor:[SKColor whiteColor] size:CGSizeMake(boxWidth,boxHeight)];
    
    Box.position = CGPointMake(x, y);
    
    Box.name = @"Box";
    
#ifdef BOX2D
    // Box2D
    
    b2BodyDef bodyDef;
    bodyDef.position.Set(Box.position.x / PIXELS_PER_METER, Box.position.y / PIXELS_PER_METER);
    //bodyDef.angle = DEG_TO_RAD(obj.rotation);
    
    b2BodyType type = b2_dynamicBody;
    
   
    
    bodyDef.type = type;
    
    bodyDef.linearDamping = 0;
    bodyDef.angularDamping = 0;
    bodyDef.allowSleep = true;
    bodyDef.awake = true;
    
    b2Body* body = world->CreateBody(&bodyDef);
    
    b2FixtureDef fixtureDef;
    b2PolygonShape polyShape;
        
        
    
    polyShape.SetAsBox(Box.size.width / 2.0 / PIXELS_PER_METER, Box.size.height / 2.0 / PIXELS_PER_METER);
    fixtureDef.shape = &polyShape;
    
    fixtureDef.density = DENSITY;
    fixtureDef.friction = FRICTION;
    fixtureDef.restitution = RESTITUTION;
    fixtureDef.isSensor = 0;
    
    body->CreateFixture(&fixtureDef);
    
    body->SetUserData((__bridge void*)Box);
    
    //body->SetAngularVelocity(6.28);
    body->SetAngularVelocity(ANGULAR_VELOCITY);
    body->SetAngularDamping(ANGULAR_DAMPING);
    body->SetLinearDamping(LINEAR_DAMPING);
    
#else
    Box.physicsBody = [SKPhysicsBody bodyWithRectangleOfSize:Box.size];
    Box.physicsBody.angularVelocity = ANGULAR_VELOCITY;
    Box.physicsBody.angularDamping = ANGULAR_DAMPING;
    Box.physicsBody.restitution = RESTITUTION;
    Box.physicsBody.density = DENSITY;
    Box.physicsBody.friction = FRICTION;
    Box.physicsBody.linearDamping = LINEAR_DAMPING;
    
#endif
    
    [self addChild:Box];
    
}


#ifdef BOX2D
void updatePhysics(double deltaT, double &accumulator, double timeStep, b2World *world) {
    int velocityIterations = 8;
    int positionIterations = 3;
    //int velocityIterations = 10;
    //int positionIterations = 10;
    
    
    if (deltaT > 0.25) {
        deltaT = 0.25;// note: max frame time to avoid spiral of death
    }
    
    accumulator += deltaT;
    
    while ( accumulator >= timeStep ) {
        if (accumulator < timeStep * 2.0) {
            // only update if on last simulation loop
            
            
            for (b2Body* b = world->GetBodyList(); b; b = b->GetNext()) {
                SKSpriteNode *sprite = (__bridge SKSpriteNode *)b->GetUserData();
                b2Vec2 position = b->GetPosition();
                
                CGPoint pos = CGPointMake(position.x*PIXELS_PER_METER, position.y*PIXELS_PER_METER);
                sprite.position = pos;
                
                float32 angle = b->GetAngle();
                
                sprite.zRotation = angle;
                
                
                //GemLog(@"(x,y,theta) = (%4.2f, %4.2f, %4.2f)\n", position.x, position.y, angle);
            }
           
            
            
        }
        
        
        world->Step(timeStep, velocityIterations, positionIterations);
        
        
        accumulator -= timeStep;
    }
    
    // interpolate remainder of update
    const double alpha = accumulator / timeStep;
    
    for (b2Body* b = world->GetBodyList(); b; b = b->GetNext()) {
        SKSpriteNode *sprite = (__bridge SKSpriteNode *)b->GetUserData();
        b2Vec2 position = b->GetPosition();
        CGPoint pos = CGPointMake((position.x * PIXELS_PER_METER +(1.0 - alpha) * sprite.position.x/PIXELS_PER_METER),
                                  (position.y * PIXELS_PER_METER + (1.0 - alpha)*sprite.position.y/PIXELS_PER_METER));
        sprite.position = pos;
        
        //NSLog(@"x = %f   y = %f", pos.x, pos.y);
        
        float32 angle = b->GetAngle();
        sprite.zRotation = alpha * angle + (1.0-alpha)*sprite.zRotation;
       
    }
    
};
#endif

-(void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event {
    /* Called when a touch begins */
    
}



-(void)update:(CFTimeInterval)currentTime {
    
    /* Called before each frame is rendered */
    if (prevTime == -1) {
        prevTime = currentTime;
    }
    double deltaT = currentTime - prevTime;
    //NSLog (@"deltaT = %f", deltaT);
    if (deltaT > 0) {
        #ifdef BOX2D
        updatePhysics(deltaT, accumulator, timeStep, world);
        
        BOOL allSleeping = YES;
        for (b2Body* b = world->GetBodyList(); b; b = b->GetNext()) {
            if (b->IsAwake()) {
                allSleeping = NO;
            }
        }
        
        if (allSleeping) {
            NSLog(@"All boxes are sleeping");
        }
        
        #else
        
        NSArray *children = self.children;
        BOOL allResting = YES;
        for (int i=0;i < [children count]; i++) {
            SKSpriteNode *box = [children objectAtIndex:i];
            SKPhysicsBody *body = box.physicsBody;
            if ( !body.resting) {
                allResting = NO;
            }
        }
        if (allResting) {
            NSLog(@"All boxes are resting");
        }

        
        #endif
        /*frameTimeTotal += deltaT;
        
        frameRate[frameCount] = 1.0 / deltaT;
        
        frameCount++;
        
        if (frameCount == MOV_AVG_COUNT) {
            double avgFrameRate = 0;
            for(int i=0;i<frameCount;i++){
                avgFrameRate += frameRate[i];
            }
            avgFrameRate = avgFrameRate / frameCount;
            
            if ((frameTimeTotal > FIRST_SAMPLE_POINT && frameTimeTotal < SECOND_SAMPLE_POINT && !frameRateDisplayed) || (frameTimeTotal > SECOND_SAMPLE_POINT && frameRateDisplayed)) {
                NSLog(@"Frame rate = %f", avgFrameRate);
                frameRateDisplayed = !frameRateDisplayed;
            }
            
            
            
            frameCount = 0;
        }*/
    }
    
    
    prevTime = currentTime;

    
}

@end
