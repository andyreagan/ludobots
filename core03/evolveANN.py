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

if __name__ == '__main__':
    runTime = 10
    numNeurons = 10
    neuronValues = np.zeros([numNeurons,runTime])
    neuronValues[:,0] = np.random.random(numNeurons)
    synapseWeights = np.random.uniform(low=-1.0,high=1.0,size=[numNeurons,numNeurons])

    
    for t in range(1,runTime):
        neuronValues[:,t] = updateNeurons(neuronValues[:,t-1],synapseWeights)

    plt.imshow(neuronValues, cmap=plt.cm.gray, aspect='auto', interpolation='nearest')
    plt.xlabel('Timestep')
    plt.ylabel('Neuron')
    plt.show()


