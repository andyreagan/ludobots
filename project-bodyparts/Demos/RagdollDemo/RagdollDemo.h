/*
Bullet Continuous Collision Detection and Physics Library
RagdollDemo
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
*/

#ifndef RAGDOLLDEMO_H
#define RAGDOLLDEMO_H

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

class RagdollDemo : public GlutDemoApplication
{
	//keep the collision shapes, for deletion/cleanup
    
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;
    
    btRigidBody* body[9]; // one main body, 4x2 leg segments
    
    btCollisionShape* geom[9];
    
    bool pause;

public:
    
	void initPhysics();

	void exitPhysics();

	virtual ~RagdollDemo()
	{
		exitPhysics();
	}

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	static DemoApplication* Create()
	{
		RagdollDemo* demo = new RagdollDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
    
    void CreateBox( int index, double x, double y, double z, double length, double height, double width) {
        
        btVector3 localInertia(0,0,0);
        
        btTransform offset; offset.setIdentity();
        offset.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
        
        btDefaultMotionState* myMotionState = new btDefaultMotionState(offset);
        
        geom[index] = new btBoxShape(btVector3(btScalar(length), btScalar(height), btScalar(width)));
        
        btRigidBody::btRigidBodyConstructionInfo rbInfo(btScalar(1.),myMotionState,geom[index],localInertia);
        body[index] = new btRigidBody(rbInfo);
        
        m_dynamicsWorld->addRigidBody(body[index]);
    }
    
    void CreateCylinderX( int index, double x, double y, double z, double xv, double yv, double zv) {
        
        btVector3 localInertia(0,0,0);
        
        btTransform offset; offset.setIdentity();
        offset.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
        
        btDefaultMotionState* myMotionState = new btDefaultMotionState(offset);
        
        geom[index] = new btCylinderShapeX(btVector3(btScalar(xv), btScalar(yv), btScalar(zv)));
        
        btRigidBody::btRigidBodyConstructionInfo rbInfo(btScalar(1.),myMotionState,geom[index],localInertia);
        body[index] = new btRigidBody(rbInfo);
        
        m_dynamicsWorld->addRigidBody(body[index]);
    }
    
    void CreateCylinderY( int index, double x, double y, double z, double xv, double yv, double zv) {
        
        btVector3 localInertia(0,0,0);
        
        btTransform offset; offset.setIdentity();
        offset.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
        
        btDefaultMotionState* myMotionState = new btDefaultMotionState(offset);
        
        geom[index] = new btCylinderShape(btVector3(btScalar(xv), btScalar(yv), btScalar(zv)));
        
        btRigidBody::btRigidBodyConstructionInfo rbInfo(btScalar(1.),myMotionState,geom[index],localInertia);
        body[index] = new btRigidBody(rbInfo);
        
        m_dynamicsWorld->addRigidBody(body[index]);
    }
    
    void CreateCylinderZ( int index, double x, double y, double z, double xv, double yv, double zv) {
        
        btVector3 localInertia(0,0,0);
        
        btTransform offset; offset.setIdentity();
        offset.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
        
        btDefaultMotionState* myMotionState = new btDefaultMotionState(offset);
        
        geom[index] = new btCylinderShapeZ(btVector3(btScalar(xv), btScalar(yv), btScalar(zv)));
        
        btRigidBody::btRigidBodyConstructionInfo rbInfo(btScalar(1.),myMotionState,geom[index],localInertia);
        body[index] = new btRigidBody(rbInfo);
        
        m_dynamicsWorld->addRigidBody(body[index]);
    }
    
    void DeleteObject( int index ) {
        m_dynamicsWorld->removeRigidBody( body[index] );
        delete body[index];
        delete geom[index];
    }
};





#endif
