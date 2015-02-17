# load previous module
# this may not be the best method once I have 10 modules
import sys
sys.path.append('/Users/andyreagan/class/2015/CSYS295evolutionary-robotics/')
from core01.hillclimber import *

# our standard imports
import numpy as np
import matplotlib.pyplot as plt

def plotSynapses(pos,n):
    for i in range(n):
        for j in range(n):
            x1,y1 = pos[:,i]
            x2,y2 = pos[:,j]
            plt.plot([x1,x2],[y1,y2],color=[0,0,0])

def plotSynapsesInhibit(pos,n,weight):
    for i in range(n):
        for j in range(n):
            x1,y1 = pos[:,i]
            x2,y2 = pos[:,j]
            if weight[i,j] > 0:
                plt.plot([x1,x2],[y1,y2],color=[0,0,0])
            else:
                plt.plot([x1,x2],[y1,y2],color=[0.8,0.8,0.8])

def plotSynapsesInhibitWidth(pos,n,weight):
    for i in range(n):
        for j in range(n):
            x1,y1 = pos[:,i]
            x2,y2 = pos[:,j]
            w = int(10*abs(weight[i,j]))+1
            if weight[i,j] > 0:
                plt.plot([x1,x2],[y1,y2],color=[0,0,0],linewidth=w)
            else:
                plt.plot([x1,x2],[y1,y2],color=[0.8,0.8,0.8],linewidth=w)

if __name__ == '__main__':
    runTime = 50
    numNeurons = 10
    neuronValues = np.zeros([numNeurons,runTime])
    neuronValues[:,0] = np.random.random(numNeurons)
    neuronPositions = np.zeros([2,numNeurons])
    
    # set the positions
    angle = 0.0 
    angleUpdate = 2 * np.pi /numNeurons 
    for i in range(0,numNeurons):
        x = np.sin(angle)
        y = np.cos(angle)
        neuronPositions[:,i] = [x,y]
        angle = angle + angleUpdate

    plt.plot(neuronPositions[0,:],neuronPositions[1,:],'ko',markerfacecolor=[1,1,1], markersize=18)
    plt.savefig('figure01.png')
    plt.close()

    plotSynapses(neuronPositions,numNeurons)
    plt.plot(neuronPositions[0,:],neuronPositions[1,:],'ko',markerfacecolor=[1,1,1], markersize=18)
    plt.savefig('figure02.png')
    plt.close()

    synapseWeights = np.random.uniform(low=-1.0,high=1.0,size=[numNeurons,numNeurons])
    plotSynapsesInhibit(neuronPositions,numNeurons,synapseWeights)
    plt.plot(neuronPositions[0,:],neuronPositions[1,:],'ko',markerfacecolor=[1,1,1], markersize=18)
    plt.savefig('figure03.png')
    plt.close()

    plotSynapsesInhibitWidth(neuronPositions,numNeurons,synapseWeights)
    plt.plot(neuronPositions[0,:],neuronPositions[1,:],'ko',markerfacecolor=[1,1,1], markersize=18)
    plt.savefig('figure04.png')
    plt.close()

    # now update the thing
    for t in range(1,runTime):
        # neuronValues[:,t] = np.random.random(numNeurons)
        for j in range(numNeurons):
            neuronValues[j,t] = np.max([0,np.min([1,np.dot(neuronValues[:,t-1],synapseWeights[j,:])])])

    plt.imshow(neuronValues, cmap=plt.cm.gray, aspect='auto', interpolation='nearest')
    plt.xlabel('Timestep')
    plt.ylabel('Neuron')
    plt.savefig('figure05.png')
    plt.close()



