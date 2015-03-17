# load previous module
# this may not be the best method once I have 10 modules
import sys
sys.path.append('/Users/andyreagan/class/2015/CSYS295evolutionary-robotics/')
from core01.hillclimber import *
from core02.ANN import *
from core03.evolveANN import *
import os
import time

# this function will interface with the executable
# given a set of synapse weights
def runBullet(parent):
    f = open('weights.csv','w')
    for i in range(len(parent)):
        f.write('{0:.3f}'.format(parent[i][0]))
        for j in range(1,len(parent[i])):
            f.write(',{0:.3f}'.format(parent[i][j]))
        f.write('\n')
    f.close()
    if os.path.isfile('distance.csv'):
        os.remove('distance.csv')
    os.system('./robot')
    while not os.path.isfile('distance.csv'):
        time.sleep(0.5)
    f = open('distance.csv','r')
    zdis = f.read().rstrip().split(',')[2]
    zdis = float(zdis)
    return zdis

def maxDistanceFitness(attempt,goal):
    return attempt

if __name__ == '__main__':
    # time to evolve for
    evoTime = 500

    numInputs = 4
    numMotors = 8
    synapseWeights = np.random.uniform(low=-1.0,high=1.0,size=[numInputs,numMotors])

    # test the runBullet guy
    # distance = runBullet(synapseWeights)
    # print(distance)

    goal = np.array([0])
    fit,genes = Evolve(synapseWeights,evoTime,runBullet,maxDistanceFitness,goal,[-1,1])


