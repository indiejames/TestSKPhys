//
//  MyScene.m
//  TestSKPhys
//
//  Created by James Norton on 9/17/13.
//  Copyright (c) 2013 James Norton. All rights reserved.
//

#import "MyScene.h"
#import <Box2D.h>

//#define BOX2D
#define NUM_BOXES (200)
#define PIXELS_PER_METER (150)
#define BOX_WIDTH (20)
#define BOX_HEIGHT (BOX_WIDTH / 2)
#define ANGULAR_VELOCITY (25)
#define ANGULAR_DAMPING (0.1)
#define DENSITY (1.0)
#define FRICTION (0.2)
#define RESTITUTION (0.2)
#define GRAVITY (-4.5)
#define MOV_AVG_COUNT (10)
#define FIRST_SAMPLE_POINT (0.5)
#define SECOND_SAMPLE_POINT (10.0)

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

-(void)addBoxes {
    float xStart = BOX_WIDTH;
    float yStart = self.size.height-BOX_HEIGHT - 30;
    
    for(int i = 0;i<NUM_BOXES;i++) {
        float x = (i % 15) * BOX_WIDTH + xStart;
        float y = yStart - (i / 15) * (BOX_HEIGHT + 10);
        [self addRockAtX:x Y:y];
        
    }
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


- (void)addRockAtX:(float )x Y:(float)y {
    
    
    
    
    SKSpriteNode *rock = [[SKSpriteNode alloc] initWithColor:[SKColor whiteColor] size:CGSizeMake(BOX_WIDTH,BOX_HEIGHT)];
    
    rock.position = CGPointMake(x, y);
    
    rock.name = @"rock";
    
#ifdef BOX2D
    // Box2D
    
    b2BodyDef bodyDef;
    bodyDef.position.Set(rock.position.x / PIXELS_PER_METER, rock.position.y / PIXELS_PER_METER);
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
        
        
    
    polyShape.SetAsBox(rock.size.width / 2.0 / PIXELS_PER_METER, rock.size.height / 2.0 / PIXELS_PER_METER);
    fixtureDef.shape = &polyShape;
    
    fixtureDef.density = DENSITY;
    fixtureDef.friction = FRICTION;
    fixtureDef.restitution = RESTITUTION;
    fixtureDef.isSensor = 0;
    
    body->CreateFixture(&fixtureDef);
    
    body->SetUserData((__bridge void*)rock);
    
    //body->SetAngularVelocity(6.28);
    body->SetAngularVelocity(ANGULAR_VELOCITY);
    body->SetAngularDamping(ANGULAR_DAMPING);
    
#else
    rock.physicsBody = [SKPhysicsBody bodyWithRectangleOfSize:rock.size];
    rock.physicsBody.angularVelocity = ANGULAR_VELOCITY;
    rock.physicsBody.angularDamping = ANGULAR_DAMPING;
    rock.physicsBody.restitution = RESTITUTION;
    rock.physicsBody.density = DENSITY;
    rock.physicsBody.friction = FRICTION;
    
#endif
    
    [self addChild:rock];
    
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
