/*
Bullet Continuous Collision Detection and Physics Library
Ragdoll Demo
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
 
2015-01-28
Modified by Andy Reagan
*/

#define CONSTRAINT_DEBUG_SIZE 0.2f

#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"

#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "RagdollDemo.h"


// Enrico: Shouldn't these three variables be real constants and not defines?

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

class RagDoll
{

	btDynamicsWorld* m_ownerWorld;
	//btCollisionShape* m_shapes[BODYPART_COUNT];
	//btRigidBody* m_bodies[BODYPART_COUNT];
	//btTypedConstraint* m_joints[JOINT_COUNT];

	btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
	{
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			shape->calculateLocalInertia(mass,localInertia);

		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		m_ownerWorld->addRigidBody(body);

		return body;
	}

public:
	RagDoll (btDynamicsWorld* ownerWorld, const btVector3& positionOffset)
		: m_ownerWorld (ownerWorld)
	{
		// Setup the geometry
		// m_shapes[BODYPART_PELVIS] = new btCapsuleShape(btScalar(0.15), btScalar(0.20));

		// Setup all the rigid bodies
		btTransform offset; offset.setIdentity();
		offset.setOrigin(positionOffset);

		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.), btScalar(0.)));
        
		// m_bodies[BODYPART_PELVIS] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_PELVIS]);

		// Now setup the constraints
		// btHingeConstraint* hingeC;
    
		btTransform localA, localB;

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.15), btScalar(0.)));
		//hingeC =  new btHingeConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_SPINE], localA, localB);
		//hingeC->setLimit(btScalar(-M_PI_4), btScalar(M_PI_2));
		//m_joints[JOINT_PELVIS_SPINE] = hingeC;
		//hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		//m_ownerWorld->addConstraint(m_joints[JOINT_PELVIS_SPINE], true);
	}

	virtual	~RagDoll ()
	{
		// int i;

		// Remove all constraints
//		for ( i = 0; i < JOINT_COUNT; ++i)
//		{
//			m_ownerWorld->removeConstraint(m_joints[i]);
//			delete m_joints[i]; m_joints[i] = 0;
//		}
//
//		// Remove all bodies and shapes
//		for ( i = 0; i < BODYPART_COUNT; ++i)
//		{
//			m_ownerWorld->removeRigidBody(m_bodies[i]);
//			
//			delete m_bodies[i]->getMotionState();
//
//			delete m_bodies[i]; m_bodies[i] = 0;
//			delete m_shapes[i]; m_shapes[i] = 0;
//		}
	}
};

static RagdollDemo* ragdollDemo;

// bool RagdollDemo::myContactProcessedCallback(btManifoldPoint& cp,
bool myContactProcessedCallback(btManifoldPoint& cp,
                                void* body0, void* body1)
{
    int *ID1, *ID2;
    btCollisionObject* o1 = static_cast<btCollisionObject*>(body0);
    btCollisionObject* o2 = static_cast<btCollisionObject*>(body1);
    // int groundID = 9;
    
    ID1 = static_cast<int*>(o1->getUserPointer());
    ID2 = static_cast<int*>(o2->getUserPointer());
    
    // Your code will go here. See the next step.
    
    // printf("ID1 = %d, ID2 = %d\n", *ID1, *ID2);
    
    ragdollDemo->touches[*ID1] = ragdollDemo->IDs[*ID2];
    ragdollDemo->touches[*ID2] = ragdollDemo->IDs[*ID1];
    
    ragdollDemo->touchPoints[*ID1] = cp.m_positionWorldOnB;
    ragdollDemo->touchPoints[*ID2] = cp.m_positionWorldOnB;
    
    return false;
}

void RagdollDemo::loadSynapsesFromFile() {
    
    FILE *ifp;
    char mode = 'r';
    
    ifp = fopen("/Users/andyreagan/class/2015/CSYS295evolutionary-robotics/final/weights.csv", &mode);
    
    if (ifp == NULL) {
        fprintf(stderr, "Can't open input file /Users/andyreagan/class/2015/CSYS295evolutionary-robotics/final/weights.csv!\n");
        exit(1);
    }
    
    double w1;
    double w2;
    double w3;
    double w4;
    double w5;
    double w6;
    double w7;
    double w8;
    double w9;
    double w10;
    double w11;
    double w12;
    double w13;
    double w14;
    int lines = 0;
    
    while (fscanf(ifp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",&w1,&w2,&w3,&w4,&w5,&w6,&w7,&w8,&w9,&w10,&w11,&w12,&w13,&w14) != EOF) {
        // printf("%f,%f,%f,%f,%f,%f,%f,%f\n",w1,w2,w3,w4,w5,w6,w7,w8);
        weights[lines][0] = w1;
        weights[lines][1] = w2;
        weights[lines][2] = w3;
        weights[lines][3] = w4;
        weights[lines][4] = w5;
        weights[lines][5] = w6;
        weights[lines][6] = w7;
        weights[lines][7] = w8;
        weights[lines][8] = w9;
        weights[lines][9] = w10;
        weights[lines][10] = w11;
        weights[lines][11] = w12;
        weights[lines][12] = w13;
        weights[lines][13] = w14;
        lines++;
    }
    
    fclose(ifp);
}


void RagdollDemo::initPhysics()
{
    
    ragdollDemo = this;
    gContactProcessedCallback = myContactProcessedCallback;
    
    for (int i=0; i<15; i++) {
        touches[i] = 0;
    }
    
    bodyLookup[0] = 3;
    bodyLookup[1] = 4;
    bodyLookup[2] = 7;
    bodyLookup[3] = 8;
    // the body parts for which we care about touches
    bodyLookup[4] = 11;
    bodyLookup[5] = 12;
    
    timeStep = 0;
    timeStepExit = 0;
    
	// Setup the basic world

	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(5.));

	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);

	m_solver = new btSequentialImpulseConstraintSolver;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

	// Setup a big ground box
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-10,0));

		btCollisionObject* fixedGround = new btCollisionObject();
		fixedGround->setCollisionShape(groundShape);
		fixedGround->setWorldTransform(groundTransform);
        // m_dynamicsWorld->addCollisionObject(fixedGround, COL_POWERUP, powerupCollidesWith);
        m_dynamicsWorld->addCollisionObject(fixedGround);
        (fixedGround)->setUserPointer( &(IDs[14]) );
	}

	// Spawn one ragdoll
	// btVector3 startOffset(1,0.5,0);
	// spawnRagdoll(startOffset);
    // startOffset.setValue(-1,0.5,0);
	// spawnRagdoll(startOffset);
    
    for (int i=0; i<15; i++) {
        IDs[i] = i;
    }
    
    // starting it at 0,0,0 puts it halfway in the ground box
    // so this definitely positions the center
    // the moving it up to 0,.2,0 gets it out of the ground
    // so the thing must have size 2,.4,2
    
    double verticalOffset = 0.0;
    
    // this starts at exactly 1 off the ground
    // and has size 1,.2,1
    CreateBox(0, 0., 1.+verticalOffset, 0., 0.5, .1, 0.5);
    
    // right upper leg
    CreateCylinder(1, 1., 1.+verticalOffset, 0., .1, 0.5, 'x');
    // left upper leg
    CreateCylinder(2, -1., 1.+verticalOffset, 0., .1, 0.5, 'x');
    // right lower leg
    CreateCylinder(3, 1.5, 0.5+verticalOffset, 0.0, .1, 0.5, 'y');
    // left lower leg
    CreateCylinder(4, -1.5, 0.5+verticalOffset, 0.0, .1, 0.5, 'y');
    CreateCylinder(5, 0., 1.+verticalOffset, 1., .1, 0.5, 'z');
    CreateCylinder(6, 0., 1.+verticalOffset, -1., .1, 0.5, 'z');
    // back lower leg
    CreateCylinder(7, 0., 0.5+verticalOffset, 1.5, .1, 0.5, 'y');
    // front lower leg
    CreateCylinder(8, 0., 0.5+verticalOffset, -1.5, .1, 0.5, 'y');
    
    // now create the pincher
    // left pincher upper (from the center of the robot)
    // it's centered at -1,1,-.5
    // and oriented in x
    // making the endpoints -0.5,1,-0.5, and -1.5,1,-0.5
    CreateCylinder(9, -1., 1.+verticalOffset, -.4, .1, 0.5, 'x');
    // right pincher upper
    CreateCylinder(10, -.4, 1.+verticalOffset, -1., .1, 0.5, 'z');
    // left pincher forearm
    CreateCylinder(11, -1.5, 1.+verticalOffset, -.55, .1, 0.2, 'z');
    // right pincher forearm
    CreateCylinder(12, -.55, 1.+verticalOffset, -1.5, .1, 0.2, 'x');
    
    // use the objectGeom number (between 0 and 1) to create a spheroid
    // CreateCapsule(objectGeom);
    int newObjectIndex = 13;
    btVector3 localInertia(0.,0.,0.);
    btTransform offset; offset.setIdentity();
    offset.setOrigin(btVector3(btScalar(-1.5), btScalar(0.75), btScalar(-1.5)));
    // offset.setOrigin(btVector3(btScalar(-1.5), btScalar(0.75), btScalar(-1.5)));
    btScalar mass(3.f);
    btDefaultMotionState* myMotionState = new btDefaultMotionState(offset);
    btCollisionShape* colShape;
    if (objectGeom > 0) {
        colShape = new btSphereShape(0.75);
    }
    else {
        colShape = new btBoxShape(btVector3(btScalar(0.75),btScalar(0.75),btScalar(0.75)));
    }
    colShape->calculateLocalInertia(mass,localInertia);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
    body[newObjectIndex] = new btRigidBody(rbInfo);
    m_dynamicsWorld->addRigidBody(body[newObjectIndex]);
    // newObject->setUserPointer( &newObjectIndex );
    (body[newObjectIndex])->setUserPointer( &(IDs[newObjectIndex]) );
    
    // trying to get these offsets right....not so easy!
    
    offsets[0] = 0;
    offsets[1] = 0;
    offsets[2] = 0;
    offsets[3] = 0;
    offsets[4] = 0;
    offsets[5] = 0;
    offsets[6] = 0;
    offsets[7] = 0;
    // what I think these should be:
//    offsets[8] = -45;
//    offsets[9] = 45;
//    offsets[10] = 45;
//    offsets[11] = -45;
    // what seems to work better...
    offsets[8] = 0;
    offsets[9] = 0;
    offsets[10] = 0;
    offsets[11] = 0;
//    offsets[8] = -90;
//    offsets[9] = 90;
//    offsets[10] = 90;
//    offsets[11] = -90;
//    offsets[8] = 90;
//    offsets[9] = -90;
//    offsets[10] = -90;
//    offsets[11] = 90;
    

    
    // left leg knee
    CreateHinge(0,1,3,1.5,1.+verticalOffset,0.0,0,0,-1,(-45.+offsets[0])*3.14159/180., (45.+offsets[0])*3.14159/180.);
    // right leg knee
    CreateHinge(1,2,4,-1.5,1.+verticalOffset,0.0,0,0,1,(-45.+offsets[1])*3.14159/180., (45.+offsets[1])*3.14159/180.);
    // far leg knee
    CreateHinge(2,5,7,0,1.+verticalOffset,1.5,1,0,0,(-45.+offsets[2])*3.14159/180., (45.+offsets[2])*3.14159/180.);
    // close leg knee
    CreateHinge(3,6,8,0,1.+verticalOffset,-1.5,-1,0,0,(-45.+offsets[3])*3.14159/180., (45.+offsets[3])*3.14159/180.);

    // left leg body
    CreateHinge(4,0,1,0.5,1.+verticalOffset,0.0,0,0,-1,(-45.+offsets[4])*3.14159/180., (45.+offsets[4])*3.14159/180.);
    // right leg body
    CreateHinge(5,0,2,-0.5,1.+verticalOffset,0.0,0,0,1,(-45.+offsets[5])*3.14159/180., (45.+offsets[5])*3.14159/180.);
    // far leg body
    CreateHinge(6,0,5,0.,1.+verticalOffset,0.5,1,0,0,(-45.+offsets[6])*3.14159/180., (45.+offsets[6])*3.14159/180.);
    // close leg body
    CreateHinge(7,0,6,0.,1.+verticalOffset,-0.5,-1,0,0,(-45.+offsets[7])*3.14159/180., (45.+offsets[7])*3.14159/180.);
    
    // body hinges for the pinchers
    CreateHinge(8,0,9,-0.5,1.+verticalOffset,-0.4,0,1,0,(-45.+offsets[8])*3.14159/180., (45.+offsets[8])*3.14159/180.);
    CreateHinge(9,0,10,-0.4,1.+verticalOffset,-0.5,0,-1,0,(-45.+offsets[9])*3.14159/180., (45.+offsets[9])*3.14159/180.);
    
    // elbow hinges for the pinchers
    CreateHinge(10,9,11,-1.5,1.+verticalOffset,-0.4,0,1,0,(-45.+offsets[10])*3.14159/180., (45.+offsets[10])*3.14159/180.);
    CreateHinge(11,10,12,-0.4,1.+verticalOffset,-1.5,0,-1,0,(-45.+offsets[11])*3.14159/180., (45.+offsets[11])*3.14159/180.);
    
    // pause = !pause;
    clientResetScene();
}

void RagdollDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    clientMove();
    
	renderme();

	glFlush();

	glutSwapBuffers();
}

void RagdollDemo::clientMove()
{
    //simple dynamics world doesn't handle fixed-time-stepping
    // float ms = getDeltaTimeMicroseconds();
    
    //float minFPS = 1000000.f/60.f;
    //float minFPS = 90000.f/60.f;
    //if (ms > minFPS)
    //	ms = minFPS;
    
    timeStepExit++;
    
    
    
    if (m_dynamicsWorld)
    {
        if (!pause || (pause && oneStep)) {
            
            // body, upper, upper, lower, lower, upper, upper, lower, lower, inner, inner, outer, outer, box, ground
            // set touches vector to zero
            for (int i=0; i<14; i++) {
                // fprintf(stdout,"%d,",touches[i]);
                touches[i] = 0;
            }
            // fprintf(stdout,"%d\n",touches[14]);
            touches[14] = 0;
            m_dynamicsWorld->stepSimulation(0.1);
            
            oneStep = !oneStep;
            
            // I seem to be able to update every timestep
            if ( timeStep%1==0 ) {
                for (int i=0; i<12; i++) {
                    double motorCommand = 0.0;
                    
                    for (int j=0; j<6; j++) {
                        motorCommand = motorCommand + weights[j][i]*touches[bodyLookup[j]];
                    }
                    
                    motorCommand = tanh(motorCommand);
                    motorCommand = motorCommand*45;
                    
                    ActuateJoint2(i, motorCommand, 0.1);
                }
                if ( streamOutput ) {
                    fprintf(stdout,"%d,%d,",touches[11],touches[12]);
                    // fprintf(stdout,"streaming output");
                    // take the final two nuerons
                    // and stream them out
                    int i = 12;
                    double output = 0.0;
                    for (int j=0; j<6; j++) {
                        output = output + weights[j][i]*touches[bodyLookup[j]];
                    }
                    fprintf(stdout,"%f,",output);
                    i = 13;
                    output = 0.0;
                    for (int j=0; j<6; j++) {
                        output = output + weights[j][i]*touches[bodyLookup[j]];
                    }
                    fprintf(stdout,"%f\n",output);
                }
            }
            timeStep++;
        }
    }
    
    if ( timeStepExit==20000 ) {
        // only save the position if we care about that...
        if ( !streamOutput ) {
            Save_Position(body[0]);
        }
        exit(0);
    }
}


void RagdollDemo::Save_Position(btRigidBody *bodypart) {
    btVector3 pos;
    pos = bodypart->getCenterOfMassPosition();
    // printf("%f,%f,%f\n",pos[0],pos[1],pos[2]);
        FILE *ofp;
        char outputFilename[] = "/Users/andyreagan/class/2015/CSYS295evolutionary-robotics/core10/distance.csv";
        ofp = fopen(outputFilename, "w");
    
        if (ofp == NULL) {
            fprintf(stderr, "Can't open output file %s!\n",
                    outputFilename);
            exit(1);
        }
    
    fprintf(ofp,"%f,%f,%f",pos[0],pos[1],pos[2]);
    fclose(ofp);
    fprintf(stdout,"%f,%f,%f",pos[0],pos[1],pos[2]);
}

void RagdollDemo::CreateBox( int index, double x, double y, double z, double length, double height, double width)
{
    
    btVector3 localInertia(0.,0.,0.);
    
    btTransform offset;
    offset.setIdentity();
    offset.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
    
    btDefaultMotionState* myMotionState = new btDefaultMotionState(offset);
    
    geom[index] = new btBoxShape(btVector3(btScalar(length), btScalar(height), btScalar(width)));
    
    btRigidBody::btRigidBodyConstructionInfo rbInfo(btScalar(1.),myMotionState,geom[index],localInertia);
    body[index] = new btRigidBody(rbInfo);
    
    // m_dynamicsWorld->addRigidBody(body[index], COL_LAND, landCollidesWith);
    m_dynamicsWorld->addRigidBody(body[index]);
    
    (body[index])->setUserPointer( &(IDs[index]) );
}

// void RagdollDemo::CreateCylinder( int index, double x, double y, double xv, double yv, double zv, char orientation) {
void RagdollDemo::CreateCylinder( int index, double x, double y, double z, double r, double len, char orientation)
{
    
    // for some reason, giving them a bump to start makes them behave normally
    // btVector3 localInertia(1.5,0.,1.5);
    // btVector3 localInertia(.01,.0,.01);
    
    btVector3 localInertia(0.,0.,0.);
    
    btTransform offset; offset.setIdentity();
    offset.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
    
    // btDefaultMotionState* myMotionState = new btDefaultMotionState(offset);
    
    btScalar	mass(1.f);
    
    switch (orientation)
    {
            /*
             cylinder is defined as following:
             *
             * - principle axis aligned along y by default, radius in x, z-value not used
             * - for btCylinderShapeX: principle axis aligned along x, radius in y direction, z-value not used
             * - for btCylinderShapeZ: principle axis aligned along z, radius in x direction, y-value not used
             *
             */
        case 'x':
        {
            // offset.getBasis().setEulerZYX(btScalar(0), btScalar(0), btScalar(M_PI_2));
            btDefaultMotionState* myMotionState = new btDefaultMotionState(offset);
            // know that the x axis is principle, z axis not used
            // geom[index] = new btCylinderShapeX(btVector3(btScalar(xv), btScalar(yv), btScalar(zv)));
            geom[index] = new btCylinderShapeX(btVector3(btScalar(len), btScalar(r), btScalar(0.)));
            // geom[index] = new btCapsuleShape(btScalar(r),btScalar(len));
            // geom[index] = new btCylinderShape(btVector3(btScalar(r), btScalar(len), btScalar(0.)));
            btCollisionShape* colShape = geom[index];
            colShape->calculateLocalInertia(mass,localInertia);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
            body[index] = new btRigidBody(rbInfo);
            // m_dynamicsWorld->addRigidBody(body[index], COL_LAND, landCollidesWith);
            m_dynamicsWorld->addRigidBody(body[index]);
            break;
        }
        case 'z':
        {
            // offset.getBasis().setEulerZYX(btScalar(M_PI_2), btScalar(0), btScalar(0));
            btDefaultMotionState* myMotionState = new btDefaultMotionState(offset);
            // geom[index] = new btCylinderShapeZ(btVector3(btScalar(xv), btScalar(yv), btScalar(zv)));
            geom[index] = new btCylinderShapeZ(btVector3(btScalar(r), btScalar(0.), btScalar(len)));
            // geom[index] = new btCapsuleShape(btScalar(r),btScalar(len));
            // geom[index] = new btCylinderShape(btVector3(btScalar(r), btScalar(len), btScalar(0.)));
            btCollisionShape* colShape = geom[index];
            colShape->calculateLocalInertia(mass,localInertia);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
            body[index] = new btRigidBody(rbInfo);
            // m_dynamicsWorld->addRigidBody(body[index], COL_LAND, landCollidesWith);
            m_dynamicsWorld->addRigidBody(body[index]);
            break;
        }
        default:
            // offset.getBasis().setEulerZYX(btScalar(0), btScalar(M_PI_2), btScalar(0));
            btDefaultMotionState* myMotionState = new btDefaultMotionState(offset);
            // geom[index] = new btCapsuleShape(btScalar(r),btScalar(len));
            // geom[index] = new btCylinderShape(btVector3(btScalar(xv), btScalar(yv), btScalar(zv)));
            geom[index] = new btCylinderShape(btVector3(btScalar(r), btScalar(len), btScalar(0.)));
            btCollisionShape* colShape = geom[index];
            colShape->calculateLocalInertia(mass,localInertia);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
            body[index] = new btRigidBody(rbInfo);
            // m_dynamicsWorld->addRigidBody(body[index], COL_LAND, landCollidesWith);
            m_dynamicsWorld->addRigidBody(body[index]);
    }
    (body[index])->setUserPointer( &(IDs[index]) );
}

void RagdollDemo::CreateHinge(int index, int body1, int body2,
                 double x, double y, double z,
                 double ax, double ay, double az,
                 double theta1, double theta2)
{
    btVector3 p(x, y, z);
    btVector3 a(ax, ay, az);
    btVector3 p1 = PointWorldToLocal(body1, p);
    btVector3 p2 = PointWorldToLocal(body2, p);
    btVector3 a1 = AxisWorldToLocal(body1, a);
    btVector3 a2 = AxisWorldToLocal(body2, a);
    joints[index] = new btHingeConstraint(*body[body1], *body[body2],
                                          p1, p2,
                                          a1, a2, false);
    joints[index]->setLimit(theta1,theta2);
    m_dynamicsWorld->addConstraint( joints[index] , true);
}

void RagdollDemo::ActuateJoint(int jointIndex, double desiredAngle,
                               double jointOffset, double timeStep)
{
    joints[jointIndex]->enableMotor(1);
    joints[jointIndex]->setMaxMotorImpulse(2);
    joints[jointIndex]->setMotorTarget(desiredAngle, 1);
}

void RagdollDemo::ActuateJoint2(int jointIndex, double desiredAngle,
                                double timeStep)
{
    double currentAngle;
    currentAngle = joints[jointIndex]->getHingeAngle();
    double maxImpulse = 0.2;
    double diff;
    diff = desiredAngle-(currentAngle+offsets[jointIndex]);
    joints[jointIndex]->enableAngularMotor(true, 20*diff, maxImpulse);
}




void RagdollDemo::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}

void RagdollDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'e':
		{
            btVector3 startOffset(0,2,0);
            break;
		}
    case 'p':
        {
            pause = !pause;
            break;
        }
    case 'o':
        {
            oneStep = !oneStep;
            break;
        }
	default:
		DemoApplication::keyboardCallback(key, x, y);
	}

	
}

void RagdollDemo::exitPhysics()
{
    DeleteObject(0);
    
	int i;

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;
	
}


void RagdollDemo::DeleteObject( int index ) {
    m_dynamicsWorld->removeRigidBody( body[index] );
    delete body[index];
    delete geom[index];
}

void RagdollDemo::DestroyHinge( int index ) {
    m_dynamicsWorld->removeConstraint( joints[index] );
    delete joints[index];
}

btVector3 RagdollDemo::PointWorldToLocal(int index, btVector3 &p) {
    btTransform local1 = body[index]->getCenterOfMassTransform().inverse();
    return local1 * p;
}

btVector3 RagdollDemo::AxisWorldToLocal(int index, btVector3 &a) {
    btTransform local1 = body[index]->getCenterOfMassTransform().inverse();
    btVector3 zero(0,0,0);
    local1.setOrigin(zero);
    return local1 * a;
}






