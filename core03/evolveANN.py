# load previous module
# this may not be the best method once I have 10 modules
import sys
sys.path.append('/Users/andyreagan/class/2015/CSYS295/')
from core01.hillclimber import *
from core02.ANN import *

import numpy as np
import matplotlib.pyplot as plt

def updateNeurons(neuronValuesPrev,synapseWeights):
    n = len(neuronValuesPrev)
    tmp = np.zeros(n)
    for j in range(n):
        tmp[j] = np.max([0,np.min([1,np.dot(neuronValuesPrev,synapseWeights[j,:])])])
    return tmp

def RMSEFitness(attempt,goal):
    # function for core03, rmse
    return np.sqrt(np.mean((attempt-goal)**2))

if __name__ == '__main__':
    # times to use testing
    runTime = 10
    numNeurons = 10

    # time to evolve for
    evoTime = 1000

    def testFun1(parent):
        # blank run
        neuronValues = np.zeros([numNeurons,runTime])
        # random initialization
        # neuronValues[:,0] = np.random.random(numNeurons)
        # 0.5 initialization
        neuronValues[:,0] = 0.5*np.ones(numNeurons)
        # run the network forward
        for t in range(1,runTime):
            neuronValues[:,t] = updateNeurons(neuronValues[:,t-1],parent)

        # return the last row
        return neuronValues[:,-1]

    synapseWeights = np.random.uniform(low=-1.0,high=1.0,size=[numNeurons,numNeurons])
        
    goal = np.array([1,0,1,0,1,0,1,0,1,0,])
    # parent,runtime,testFun,fitnessFun,match
    fit,genes = Evolve(synapseWeights,evoTime,testFun1,RMSEFitness,goal,[-1,1])

    # lets plot the evolution of the first and last ANN
    neuronValues = np.zeros([numNeurons,runTime])
    neuronValues[:,0] = 0.5*np.ones(numNeurons)
    for t in range(1,runTime):
        neuronValues[:,t] = updateNeurons(neuronValues[:,t-1],genes[:,:,0])
    plt.imshow(neuronValues, cmap=plt.cm.gray, aspect='auto', interpolation='nearest')
    plt.xlabel('Timestep')
    plt.ylabel('Neuron')
    # plt.show()
    plt.savefig("figure01.png")
    plt.close()

    # lets plot the evolution of the first and last ANN
    neuronValues = np.zeros([numNeurons,runTime])
    neuronValues[:,0] = 0.5*np.ones(numNeurons)
    for t in range(1,runTime):
        neuronValues[:,t] = updateNeurons(neuronValues[:,t-1],genes[:,:,-1])
    plt.imshow(neuronValues, cmap=plt.cm.gray, aspect='auto', interpolation='nearest')
    plt.xlabel('Timestep')
    plt.ylabel('Neuron')
    # plt.show()
    plt.savefig("figure02.png")
    plt.close()

    plt.plot(range(evoTime),fit)
    plt.xlabel('Timestep')
    plt.ylabel('Fitness')
    plt.savefig("figure03.png")
    plt.close()




    #################################################################
    ## second design problem

    def testFun2(parent):
        # blank run
        neuronValues = np.zeros([numNeurons,runTime])
        # random initialization
        # neuronValues[:,0] = np.random.random(numNeurons)
        # 0.5 initialization
        neuronValues[:,0] = 0.5*np.ones(numNeurons)
        # run the network forward
        for t in range(1,runTime):
            neuronValues[:,t] = updateNeurons(neuronValues[:,t-1],parent)

        # return the last row
        return neuronValues

    synapseWeights = np.random.uniform(low=-1.0,high=1.0,size=[numNeurons,numNeurons])

    # terrible
    e = np.array([1,0,1,0,1,0,1,0,1,0,])    
    o = np.array([0,1,0,1,0,1,0,1,0,1,])
    goal = np.array([e,o,e,o,e,o,e,o,e,o,])

    # parent,runtime,testFun,fitnessFun,match
    fit,genes = Evolve(synapseWeights,evoTime,testFun2,RMSEFitness,goal,[-1,1])

    # lets plot the evolution of the first and last ANN
    neuronValues = np.zeros([numNeurons,runTime])
    neuronValues[:,0] = 0.5*np.ones(numNeurons)
    for t in range(1,runTime):
        neuronValues[:,t] = updateNeurons(neuronValues[:,t-1],genes[:,:,0])
    plt.imshow(neuronValues, cmap=plt.cm.gray, aspect='auto', interpolation='nearest')
    plt.xlabel('Timestep')
    plt.ylabel('Neuron')
    # plt.show()
    plt.savefig("figure04.png")
    plt.close()

    # lets plot the evolution of the first and last ANN
    neuronValues = np.zeros([numNeurons,runTime])
    neuronValues[:,0] = 0.5*np.ones(numNeurons)
    for t in range(1,runTime):
        neuronValues[:,t] = updateNeurons(neuronValues[:,t-1],genes[:,:,-1])
    plt.imshow(neuronValues, cmap=plt.cm.gray, aspect='auto', interpolation='nearest')
    plt.xlabel('Timestep')
    plt.ylabel('Neuron')
    # plt.show()
    plt.savefig("figure05.png")
    plt.close()

    plt.plot(range(evoTime),fit)
    plt.xlabel('Timestep')
    plt.ylabel('Fitness')
    plt.savefig("figure06.png")
    plt.close()



