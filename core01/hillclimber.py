import numpy as np
import matplotlib.pyplot as plt

def MatrixCreate(down,across):
    return np.zeros([down,across])

def MatrixRandomize(matrix):
    return np.random.random(np.shape(matrix))

def MatrixPerturb(matrix, p):
    # perturb the given matrix, replacing entries with probablity p
    replaces = np.random.choice(2,[1,50],p=[1-p,p])
    return matrix-np.multiply(matrix,replaces)+np.multiply(replaces,np.random.random(np.shape(matrix)))

def Fitness(matrix,match=[]):
    # function for core01
    return np.mean(matrix)
    # function for core03, rmse
    # return np.sqrt(np.mean((matrix-match)**2))

def Evolve(parent,runtime):
    fit = np.zeros(runtime)
    genes = np.zeros([np.shape(parent)[0],np.shape(parent)[1],runtime])
    for currentGeneration in range(runtime):
        parentFitness = Fitness(parent)
        fit[currentGeneration] = parentFitness
        genes[:,:,currentGeneration] = parent
        child = MatrixPerturb(parent, .05) 
        childFitness = Fitness(child)
        if childFitness > parentFitness:
            parent = child
            parentFitness = childFitness

    return fit,genes

if __name__ == '__main__':
    runtime = 5000

    # first just plot 1
    parent = MatrixCreate(1, 50) 
    parent = MatrixRandomize(parent) 
    fit,genes = Evolve(parent,runtime)
    plt.plot(range(runtime),fit)
    plt.xlabel("Generation")
    plt.ylabel("Fitness")
    plt.savefig("figure01.png")
    plt.close()

    # now plot 5
    nruns = 5
    fits = np.zeros([nruns,runtime])

    for i in range(nruns):
        parent = MatrixCreate(1, 50) 
        parent = MatrixRandomize(parent) 
        fit,genes = Evolve(parent,runtime)
        fits[i,:] = fit
        plt.plot(range(runtime),fit)

    plt.xlabel("Generation")
    plt.ylabel("Fitness")
    plt.savefig("figure02.png")
    plt.close()
    
    # first just plot 1
    parent = MatrixCreate(1, 50) 
    parent = MatrixRandomize(parent) 
    fit,genes = Evolve(parent,runtime)
    plt.imshow(genes[0,:,:], cmap=plt.cm.gray, aspect='auto', interpolation='nearest')
    plt.xlabel("Generation")
    plt.ylabel("Gene")
    plt.savefig("figure03.png")
    plt.close()

