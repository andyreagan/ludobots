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
		btHingeConstraint* hingeC;
    
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
		int i;

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




void RagdollDemo::initPhysics()
{
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
		m_dynamicsWorld->addCollisionObject(fixedGround);

	}

	// Spawn one ragdoll
	// btVector3 startOffset(1,0.5,0);
	// spawnRagdoll(startOffset);
    // startOffset.setValue(-1,0.5,0);
	// spawnRagdoll(startOffset);
    
    CreateBox(0, 0., 2., 0., 1., .2, 1.);
    CreateCylinderX(1, 2.0, 2., 0., 1., .2, 0.);
    CreateCylinderX(2, -2.0, 2., 0., 1., .2, 0.);
    CreateCylinderY(3, 3., 1.0, 0., .2, 1., .2);
    CreateCylinderY(4, -3., 1.0, 0., .2, 1., .2);
    CreateCylinderZ(5, 0., 2., -2.0, .2, 0., 1.);
    CreateCylinderZ(6, 0., 2., 2.0, .2, 0., 1.);
    CreateCylinderY(7, 0., 1.0, 3., .2, 1., .2);
    CreateCylinderY(8, 0., 1.0, -3., .2, 1., .2);
    pause = !pause;
    clientResetScene();
}

void RagdollDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	float minFPS = 1000000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;

	if (m_dynamicsWorld)
	{
        if (!pause) {
            m_dynamicsWorld->stepSimulation(ms / 1000000.f);
        }
		
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();


	}

	renderme(); 

	glFlush();

	glutSwapBuffers();
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







