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
#include "GLDebugDrawer.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
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
    
    btHingeConstraint* joints[8];
    
    bool oneStep;
    
    bool pause;
    
    double offsets[8];
    
    
    
    int bodyLookup[4];
    
    long timeStep;
    
    long timeStepExit;
    
//    // need to do some collision masking
//    // and I'm going to ignore when body parts collide with one another
//    // reference:
//    // http://bulletphysics.org/mediawiki-1.5.8/index.php/Collision_Filtering
//    #define BIT(x) (1<<(x))
//    enum collisiontypes {
//        COL_NOTHING = 0, // 00000000
//        COL_LOWER_LEG = BIT(0), // 00000001
//        COL_UPPER_LEG = BIT(1), // 00000010
//        COL_BODY = BIT(2), // 00000011
//        COL_LAND = BIT(6), //00000111
//    };
//    
//    int nothingCollidesWith = COL_NOTHING;
//    int lowerLegCollidesWith = COL_LOWER_LEG;
//    int upperLegCollidesWith = COL_UPPER_LEG;
//    int bodyCollidesWith = COL_BODY;
//    int landCollidesWith = COL_LAND;
    
    int IDs[10];
    
public:
    
    // make public so I can fill from stdin
    double weights[4][8];
    
    // to store the weights as they come in
    // ...could just go straigt into weights
    double weightsLinear[32];
    
    // whether to stream the two output neurons to stdout
    bool streamOutput = false;
    
    int touches[10];
    
    btVector3 touchPoints[10];
    
	void initPhysics();

	void exitPhysics();

	virtual ~RagdollDemo()
	{
		exitPhysics();
	}
    
    virtual void clientMove();

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
    
    void CreateBox( int index, double x, double y, double z, double length, double height, double width);
    
    // void CreateCylinder( int index, double x, double y, double z, double xv, double yv, double zv, char orientation);
    void CreateCylinder( int index, double x, double y, double z, double r, double len, char orientation);
    
    void CreateHinge(int index, int body1, int body2, double x, double y, double z, double ax, double ay, double az, double theta1, double theta2);
    
    void DeleteObject( int index );
    
    void DestroyHinge( int index );
    
    btVector3 PointWorldToLocal(int index, btVector3 &p);
    
    btVector3 AxisWorldToLocal(int index, btVector3 &a);
    
    void Save_Position(btRigidBody *bodypart);
    
    void loadSynapsesFromFile();
    
    void ActuateJoint(int jointIndex, double desiredAngle,
                      double jointOffset, double timeStep);
    
    void ActuateJoint2(int jointIndex, double desiredAngle,
                      double timeStep);
    
    // bool myContactProcessedCallback(btManifoldPoint& cp,
    //                                 void* body0, void* body1);
    
    virtual void renderme() {
        extern GLDebugDrawer gDebugDrawer;
        // Call the parent method.
        GlutDemoApplication::renderme();
        // Make a circle with a 0.9 radius at (0,0,0)
        // with RGB color (1,0,0).
        // gDebugDrawer.drawSphere(btVector3(0.,0.,0.), 0.9, btVector3(1., 0., 0.));
        
        // draw the red circles at contact points
//        for (int i=0; i<10; i++) {
//            if (touches[i]) {
//                gDebugDrawer.drawSphere(touchPoints[i], 0.2, btVector3(1., 0., 0.));
//            }
//        }
    }
};





#endif
