# load previous module
# this may not be the best method once I have 10 modules
import sys
# sys.path.append('/Users/andyreagan/class/2015/CSYS295evolutionary-robotics/')
# just load them in directly...
import os
import time
import numpy as np
import subprocess
import csv
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

def Evolve(parent,runtime,testFun,fitnessFun,match,prange,keepallgenes=False):
    fit = np.zeros(runtime)
    if keepallgenes:
        genes = np.zeros([np.shape(parent)[0],np.shape(parent)[1],runtime])
    else:
        genes = np.zeros([np.shape(parent)[0],np.shape(parent)[1]])
    # since the testFun has gotten costly, don't run each time
    runparent = True
    for currentGeneration in range(runtime):
        # only test the performance of the parent if we need to
        if runparent:
            # print('running parent')
            perf = testFun(parent)
            parentFitness = fitnessFun(perf,match)
        fit[currentGeneration] = parentFitness
        if keepallgenes:
            genes[:,:,currentGeneration] = parent
        else:
            genes = parent
        child = MatrixPerturb(parent, .05, prange)
        # now run the child
        # print('running child')
        childperf = testFun(child)
        childFitness = fitnessFun(childperf,match)

        # print('{0}\t{1}\t{2}'.format(currentGeneration,perf,childperf))
        print('{0}\t{1:6.3f},{2:6.3f},{3:6.3f}\t{4:.3f},{5:6.3f},{6:6.3f}\t{7:6.3f}\t{8:6.3f}\n'.format(currentGeneration,perf[0],perf[1],perf[2],childperf[0],childperf[1],childperf[2],parentFitness,childFitness))

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
def runBulletStdin(parent,objectGeom=1.0,offset=0.0,sizeScaling=1.0):
    # command = './robot --headless'
    # command = './robot --headless --objectGeom {1} --streamoutput --synapses {0}'.format(' '.join(map(str,parent.flat)),objectGeom)
    command = './robot --headless --objectGeom {1} --objectOffset {2} --objectScaling {3} --streamoutput --synapses {0}'.format(' '.join(map(str,parent.flat)),objectGeom,offset,sizeScaling)
    # command = './robot --objectGeom {1} --streamoutput --synapses {0}'.format(' '.join(map(str,parent.flat)),objectGeom)
    # print(command)
    # os.system(command)
    # proc = subprocess.Popen(command,shell=True,stderr=subprocess.PIPE,stdout=subprocess.PIPE)
    proc = subprocess.Popen(command.split(" "),shell=False,stderr=subprocess.PIPE,stdout=subprocess.PIPE)
    # print(proc)
    # distance = proc.communicate()
    raw = proc.communicate()[0]
    outputList = raw.decode('utf8').split('\n')
    # print(outputList[:10])
    # print(outputList[-10:])
    outputArray = np.array([list(map(float,x.split(','))) for x in outputList[:-1]])
    outputTouches = outputArray[:,0:2]
    outputNeurons = outputArray[:,2:4]
    outputDistance = outputArray[:,4]
    # print(distance)
    # zdis = float(distance.rstrip().split(',')[1])
    # return zdis
    return outputNeurons,outputTouches,outputDistance

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
        # print('overlap on the bottom')
        return shorterline[1]-longerline[0]
    # fully overlapping
    elif shorterline[1] < longerline[1] and shorterline[0] > longerline[0]:
        # print('fully overlapping')
        return shorterline[1]-shorterline[0]
    # overlap on the top
    elif shorterline[1] > longerline[1] and shorterline[0] > longerline[0]:
        # print('overlap on the top')
        return longerline[1]-shorterline[0]
    else:
        return 0

def computeOverlap(result1,result2):
    # now compute the bounding boxes, and their overlap
    bbox1 = np.array([result1.min(axis=(0)),result1.max(axis=(0))]).flatten()
    bbox2 = np.array([result2.min(axis=(0)),result2.max(axis=(0))]).flatten()

    # print(bbox1)
    # print(bbox2)
    # # of the form [x1 y1 x2 y2]
    # print('bbox1 area: {0}'.format((bbox1[2]-bbox1[0])*(bbox1[3]-bbox1[1])))
    # print('bbox2 area: {0}'.format((bbox2[2]-bbox2[0])*(bbox2[3]-bbox2[1])))

    overlapy = overlap([bbox1[1],bbox1[3]],[bbox2[1],bbox2[3]])
    # print('overlap in y: {0}'.format(overlapy))

    overlapx = overlap([bbox1[0],bbox1[2]],[bbox2[0],bbox2[2]])
    # print('overlap in x: {0}'.format(overlapx))

    bboxoverlap = overlapx*overlapy
    print('bbox overlap area: {0}'.format(bboxoverlap))

    return bboxoverlap



def separateBBoxTestFun(synapses):
    result1neurons,result1touches,result1dis = runBulletStdin(synapses,objectGeom=1)
    result2neurons,result2touches,result2dis = runBulletStdin(synapses,objectGeom=0)
    a = computeOverlap(result1neurons,result2neurons)
    f = np.sum(result1touches==13)/(result1touches.size*2)+np.sum(result2touches==13)/(result2touches.size*2)
    # f = f*4
    dmax = 3.
    ntrials = 2.
    davg = np.mean(np.array([np.mean(result1dis),np.mean(result2dis)])-1.5)
    d = 1-davg/dmax
    print('percentage contact time: {0:.3f}'.format(f))
    print('bbox overlap area: {0:.3f}'.format(a))
    print('average distance: {0:.3f}'.format(d))
    return (a,f,d)

def multipleTrialsTestFun(synapses,trials=[[1.0,0.0],[0.0],[1.0]]):
    # trials: the parameters for all of the different trials
    # trials = [[1.0,0.0],[-0.05,0.05],[1.0]]

    trial_sizes = list(map(len,trials))
    print(trial_sizes)
    num_trials = np.cumproduct(trial_sizes)[-1]
    print(num_trials)
    trial_results = [None for i in range(num_trials)]
    print(trial_results)
    for i,objectGeom in zip(range(trial_sizes[0]),trials[0]):
        for j,offset in zip(range(trial_sizes[1]),trials[1]):
            for k,sizeScaling in zip(range(trial_sizes[2]),trials[2]):
                neurons,touches,dis = runBulletStdin(synapses,objectGeom=objectGeom,offset=offset,sizeScaling=sizeScaling)
                trial_results[i+2*j+4*k] = (neurons,touches,dis)
    print([list(map(np.shape,x)) for x in trial_results])
    object1neurons = np.reshape(np.array(list(map(lambda x: x[0],trial_results[0::2]))),[neurons.shape[0]*num_trials/2,2])
    print(object1neurons.shape)
    object2neurons = np.reshape(np.array(list(map(lambda x: x[0],trial_results[1::2]))),[neurons.shape[0]*num_trials/2,2])
    a = computeOverlap(object1neurons,object2neurons)
    f = np.sum(touches==13)/(touches.size*2)+np.sum(touches==13)/(touches.size*2)
    dmax = 3.
    davg = np.mean(np.array([np.mean(dis),np.mean(dis)])-1.5)
    d = 1-davg/dmax
    print('percentage contact time: {0:.3f}'.format(f))
    print('bbox overlap area: {0:.3f}'.format(a))
    print('average distance: {0:.3f}'.format(d))
    return (a,f,d)

def touchingAndDistanceFitness(attempt,goal):
    f1 = attempt[1] # touching
    f3 = attempt[2] # distance
    if f1 > 0.6 and f3 > 0.8:
        f2 = 1-attempt[0]/2000.
    else:
        f2 = 0
    return f1+f2+f3

def distanceFitness(attempt,goal):
    f1 = attempt[2] # distance
    if f1 > 0.8:
        f2 = 1-attempt[0]/2000.
    else:
        f2 = 0
    return f1+f2

def touchingFitness(attempt,goal):
    f1 = attempt[1] # touching fraction
    if f1 > 0.8:
        f2 = 1-attempt[0]/2000.
    else:
        f2 = 0
    return f1+f2

def maxDistanceFitness(attempt,goal):
    return attempt

def simpleRun():
    evoTime = 1000
    numInputs = 8
    numOutputs = 14
    synapseWeights = np.random.uniform(low=-1.0,high=1.0,size=[numInputs,numOutputs])
    fit,genes = Evolve(synapseWeights,evoTime,multipleTrialsTestFun,touchingFitness,0.0,[-1,1])
    return fit,genes

def onePerturbation():
    evoTime = 1000
    numInputs = 8
    numOutputs = 14
    synapseWeights = np.random.uniform(low=-1.0,high=1.0,size=[numInputs,numOutputs])
    def testFun(x):
        return multipleTrialsTestFun(x,trials=[[1.0,-1.0],[-0.3,0.5],[1.0]])
    fit,genes = Evolve(synapseWeights,evoTime,testFun,touchingFitness,0.0,[-1,1])
    return fit,genes

def twoPerturbations():
    evoTime = 1000
    numInputs = 8
    numOutputs = 14
    synapseWeights = np.random.uniform(low=-1.0,high=1.0,size=[numInputs,numOutputs])
    def testFun(x):
        return multipleTrialsTestFun(x,trials=[[1.0,0.0],[-0.3,0.5],[.6,1.25]])
    fit,genes = Evolve(synapseWeights,evoTime,testFun,touchingFitness,0.0,[-1,1])
    return fit,genes

if __name__ == '__main__':
    # time to evolve for
    evoTime = 1000

    numInputs = 8
    numOutputs = 14
    synapseWeights = np.random.uniform(low=-1.0,high=1.0,size=[numInputs,numOutputs])

    # goal = np.array([0])
    # fit,genes = Evolve(synapseWeights,evoTime,runBullet,maxDistanceFitness,goal,[-1,1])
    goal = np.array([0])

    test = False

    if test:
        # test the runBullet guy
        result1 = runBulletStdin(synapseWeights)
        print(result1.shape)
        # test it with the other shape
        result2 = runBulletStdin(np.random.uniform(low=-1.0,high=1.0,size=[numInputs,numOutputs]))
        print(result2.shape)
        # now compute the bounding boxes, and their overlap
        bbox1 = np.array([result1.min(axis=(0)),result1.max(axis=(0))]).flatten()
        bbox2 = np.array([result2.min(axis=(0)),result2.max(axis=(0))]).flatten()
        print(bbox1)
        print(bbox2)
        # of the form [x1 y1 x2 y2]
        print('bbox1 area: {0}'.format((bbox1[2]-bbox1[0])*(bbox1[3]-bbox1[1])))
        print('bbox2 area: {0}'.format((bbox2[2]-bbox2[0])*(bbox2[3]-bbox2[1])))
    
        overlapy = overlap([bbox1[1],bbox1[3]],[bbox2[1],bbox2[3]])
        print('overlap in y: {0}'.format(overlapy))
        overlapx = overlap([bbox1[0],bbox1[2]],[bbox2[0],bbox2[2]])
        print('overlap in x: {0}'.format(overlapx))
        bboxoverlap = overlapx*overlapy
        print('bbox overlap area: {0}'.format(bboxoverlap))
    
        fig = plt.figure(figsize=(10,10))
        ax = fig.add_axes([0.2,0.2,0.7,0.7])
        ax.plot(result1[:,0],result1[:,1],'b')
        ax.plot(result2[:,0],result2[:,1],'r')
        ax.add_patch(Rectangle(bbox1[0:2],bbox1[2]-bbox1[0],bbox1[3]-bbox1[1],color=[0.1,0.1,0.1,0.5]))
        ax.add_patch(Rectangle(bbox2[0:2],bbox2[2]-bbox2[0],bbox2[3]-bbox2[1],color=[0.1,0.1,0.1,0.5]))
        mysavefig('initial-bbox-test.png')
        plt.show()

    # fit,genes = Evolve(synapseWeights,evoTime,separateBBoxTestFun,touchingFitness,goal,[-1,1])
    fit,genes = Evolve(synapseWeights,3,multipleTrialsTestFun,touchingFitness,goal,[-1,1])
    print(genes.shape)
    experimentDesc = 'touching-staticObject-noScaling'

    f = open('{0}-finalSynapses-{1}.csv'.format(datetime.datetime.strftime(datetime.datetime.now(),'%Y-%m-%d-%H-%M'),experimentDesc),'w')
    csv_writer = csv.writer(f)
    csv_writer.writerows(genes)
    f.close()
    print(genes)
    plt.plot(range(evoTime),fit)
    plt.xlabel('generation')
    plt.ylabel('fitness')
    plt.title('fitness curve for armed quadraped')
    mysavefig('{0}.png'.format(experimentDesc))
    plt.show()




