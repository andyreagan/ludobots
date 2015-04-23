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
#include <iostream>
#include <string.h>

GLDebugDrawer gDebugDrawer;

int main(int argc,char* argv[])
{
    RagdollDemo demoApp;
    
    // number of input arguments:
    // printf("argc = %d\n\n",argc);

    // don't use input flag...
    bool inputflag = false;
    // keep track of headless mode
    bool headless = false;
    // keep track of whether to open them from the file
    bool synapsesLoaded = false;
    
    // okay, chars are a huge pain
    // use standard lib string
    std::string dash = "--";
    std::string input;
    
    demoApp.objectGeom = -1.0;
    
    for (int i=0; i<argc; i++) {
        // check the count
        // fprintf(stdout,"i = %d\n",i);
        // convert to a string
        input = argv[i];
        // print the converted input
        // std::cout << input << "\n";
        // std::cout << input.substr(0,2) << "\n";
        if (input.substr(0,2) == "--") {
            inputflag = true;
            if (input.substr(2) == "headless") {
                headless = true;
            }
            if (input.substr(2) == "objectGeom") {
                i++;
                demoApp.objectGeom = atof(argv[i]);
            }
            if (input.substr(2) == "pause") {
                demoApp.pause = true;
            }
            if (input.substr(2) == "streamoutput") {
                // printf("streaming output");
                demoApp.streamOutput = true;
            }
            if (input.substr(2) == "synapses") {
                int j = 0;
                // while (!inputflag) {
                // we know there are 32...
                while (j<84) {
                    // jump through i....yay c++!
                    i++;
                    // check the input...
                    // input = argv[i];
                    demoApp.weightsLinear[j] = atof(argv[i]);
                    // fprintf(stdout,"%f\n",demoApp.weightsLinear[j]);
                    j++;
                }
                // convert the linear weights into a matrix
                // could do this straight from stdin, but oh well
                for (int j=0; j<84; j++) {
                    int ji = floor(j/14);
                    int jj = j-ji*14;
                    // check that these indices are correct:
                    // std::cout << j << "\n";
                    // std::cout << ji << "," << jj << "\n";
                    demoApp.weights[ji][jj] = demoApp.weightsLinear[j];
                }
                // if all of that worked:
                synapsesLoaded = true;
            }
        }
    }
    
    demoApp.initPhysics();
    demoApp.getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);
    
    if ( !synapsesLoaded ) {
        std::cout << "loading synapses from file" << "\n";
        demoApp.loadSynapsesFromFile();
    }
    
    if ( headless ) {
        while (1) demoApp.clientMove();
        return 0;
    }
    else {
        return glutmain(argc,argv,640,480,"Bullet Physics Demo by Andy Reagan. http://bulletphysics.com",&demoApp);
    }
}