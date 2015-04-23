# load previous module
# this may not be the best method once I have 10 modules
import sys
# sys.path.append('/Users/andyreagan/class/2015/CSYS295evolutionary-robotics/')
# just load them in directly...
import os
import time
import numpy as np
import subprocess

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

import datetime
def mysavefig(name,date=True):
    if date:
        plt.savefig('{0}-{1}'.format(datetime.datetime.strftime(datetime.datetime.now(),'%Y-%m-%d-%H-%M'),name))
    else:
        plt.savefig('{1}'.format(name))

def MatrixCreate(down,across):
    return np.zeros([down,across])

def MatrixRandomize(matrix):
    return np.random.random(np.shape(matrix))

def MatrixPerturb(matrix, p, prange):
    # perturb the given matrix, replacing entries with probablity p
    replaces = np.random.choice(2,np.shape(matrix),p=[1-p,p])
    return matrix-np.multiply(matrix,replaces)+np.multiply(replaces,np.random.uniform(low=prange[0],high=prange[1],size=np.shape(matrix)))

def Core01Fitness(matrix,match):
    # function for core01
    return np.mean(matrix)

def Evolve(parent,runtime,testFun,fitnessFun,match,prange):
    fit = np.zeros(runtime)
    genes = np.zeros([np.shape(parent)[0],np.shape(parent)[1],runtime])
    # since the testFun has gotten costly, don't run each time
    runparent = True
    for currentGeneration in range(runtime):
        # only test the performance of the parent if we need to
        if runparent:
            # print('running parent')
            perf = testFun(parent)
            parentFitness = fitnessFun(perf,match)
        fit[currentGeneration] = parentFitness
        genes[:,:,currentGeneration] = parent
        child = MatrixPerturb(parent, .05, prange)
        # now run the child
        # print('running child')
        childperf = testFun(child)
        childFitness = fitnessFun(childperf,match)

        print('{0}\t{1}\t{2}'.format(currentGeneration,perf,childperf))
        # print('{0}\t{1}\t{2}\t{3}\t{4}\n'.format(currentGeneration,perf,childperf,parentFitness,childFitness))

        # if we accept the child as the new parent
        if childFitness > parentFitness:
            # print('new parent accepted')
            perf = childperf
            parent = child
            parentFitness = childFitness
            runparent = False
        else:
            runparent = False 

    return fit,genes

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

def updateNeurons(neuronValuesPrev,synapseWeights):
    n = len(neuronValuesPrev)
    tmp = np.zeros(n)
    for j in range(n):
        tmp[j] = np.max([0,np.min([1,np.dot(neuronValuesPrev,synapseWeights[j,:])])])
    return tmp

def RMSEFitness(attempt,goal):
    # function for core03, rmse
    return np.sqrt(np.mean((attempt-goal)**2))

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
    f.close()
    zdis = float(zdis)
    return zdis

# this function will interface with the executable
# given a set of synapse weights
def runBulletStdin(parent):
    command = './robot --headless'
    command = './robot --headless --objectGeom 1 --streamoutput --synapses {0}'.format(' '.join(map(str,parent.flat)))
    print(command)
    # os.system(command)
    # proc = subprocess.Popen(command,shell=True,stderr=subprocess.PIPE,stdout=subprocess.PIPE)
    proc = subprocess.Popen(command.split(" "),shell=False,stderr=subprocess.PIPE,stdout=subprocess.PIPE)
    # print(proc)
    # distance = proc.communicate()
    raw = proc.communicate()[0]
    outputList = raw.decode('utf8').split('\n')
    # print(outputList[:10])
    # print(outputList[-10:])
    outputNeurons = [list(map(float,x.split(','))) for x in outputList[:-1]]
    # print(distance)
    # zdis = float(distance.rstrip().split(',')[1])
    # return zdis
    return np.array(outputNeurons)

def maxDistanceFitness(attempt,goal):
    return attempt

if __name__ == '__main__':
    # time to evolve for
    evoTime = 500

    numInputs = 6
    numMotors = 14
    synapseWeights = np.random.uniform(low=-1.0,high=1.0,size=[numInputs,numMotors])

    # test the runBullet guy
    result1 = runBulletStdin(synapseWeights)
    print(result1.shape)

    # test it with the other shape
    result2 = runBulletStdin(np.random.uniform(low=-1.0,high=1.0,size=[numInputs,numMotors]))
    print(result2.shape)

    # now compute the bounding boxes, and their overlap
    bbox1 = np.array([result1.min(axis=(0)),result1.max(axis=(0))]).flatten()
    bbox2 = np.array([result2.min(axis=(0)),result2.max(axis=(0))]).flatten()
    print(bbox1)
    print(bbox2)
    # of the form [x1 y1 x2 y2]
    print('bbox1 area: {0}'.format((bbox1[2]-bbox1[0])*(bbox1[3]-bbox1[1])))
    print('bbox2 area: {0}'.format((bbox2[2]-bbox2[0])*(bbox2[3]-bbox2[1])))

    def overlap(line1,line2):
        # take the smaller line, and try to place it
        # on top of the bigger line
        len1 = line1[1]-line1[0]
        len2 = line2[1]-line2[0]
        if len1 > len2:
            longerline = line1
            shorterline = line2
        else:
            longerline = line2
            shorterline = line1
        # there are three cases
        # overlap on the bottom
        if shorterline[1] > longerline[0] and shorterline[0] < longerline[0]:
            print('overlap on the bottom')
            return shorterline[1]-longerline[0]
        # fully overlapping
        elif shorterline[1] < longerline[1] and shorterline[0] > longerline[0]:
            print('fully overlapping')
            return shorterline[1]-shorterline[0]
        # overlap on the top
        elif shorterline[1] > longerline[1] and shorterline[0] > longerline[0]:
            print('overlap on the top')
            return longerline[1]-shorterline[0]
        else:
            return 0

    overlapy = overlap([bbox1[1],bbox1[3]],[bbox2[1],bbox2[3]])
    print('overlap in y: {0}'.format(overlapy))
    overlapx = overlap([bbox1[0],bbox1[2]],[bbox2[0],bbox2[2]])
    print('overlap in x: {0}'.format(overlapx))
    bboxoverlap = overlapx*overlapy
    print('bbox overlap area: {0}'.format(bboxoverlap))

    # fig = plt.figure(figsize=(10,10))
    # ax = fig.add_axes([0.2,0.2,0.7,0.7])
    # # plt.xkcd()
    # ax.plot(result1[:,0],result1[:,1],'b')
    # ax.plot(result2[:,0],result2[:,1],'r')
    # ax.add_patch(Rectangle(bbox1[0:2],bbox1[2]-bbox1[0],bbox1[3]-bbox1[1],color=[0.1,0.1,0.1,0.5]))
    # ax.add_patch(Rectangle(bbox2[0:2],bbox2[2]-bbox2[0],bbox2[3]-bbox2[1],color=[0.1,0.1,0.1,0.5]))
    # mysavefig('initial-bbox-test.png')
    # plt.show()

    # overlap = bbox
    
    # goal = np.array([0])
    # fit,genes = Evolve(synapseWeights,evoTime,runBullet,maxDistanceFitness,goal,[-1,1])




