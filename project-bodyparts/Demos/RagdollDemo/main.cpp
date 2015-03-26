/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "RagdollDemo.h"
#include "GlutStuff.h"  
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

GLDebugDrawer gDebugDrawer;

int main(int argc,char* argv[])
{
    RagdollDemo demoApp;
    
    demoApp.initPhysics();
    demoApp.getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);
    
    // printf("argc = %d\n\n",argc);
    
    if ( argc > 1 ) {
//        printf("argv[0] = %s\n",argv[0]);
//        printf("\n");
//        printf("argv[1] = %s\n",argv[1]);
//        printf("\n");
        if ( strcmp(argv[1],"-headless") == 0) {
        // if ( argv[1] == "-headless" ) {
            // fprintf(stdout,"running headless\n\n");
            while (1) demoApp.clientMove();
            return 0;
        }
        else {
            printf("not headless, running with display\n\n");
            return glutmain(argc,argv,640,480,"Bullet Physics Demo by Andy Reagan. http://bulletphysics.com",&demoApp);
        }
    }
    else {
        // printf("less than 1 argc, running with display\n\n");
        return glutmain(argc,argv,640,480,"Bullet Physics Demo by Andy Reagan. http://bulletphysics.com",&demoApp);
    }
}