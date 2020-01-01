import os
from matplotlib import pyplot as plt

sigmas = ['0.010000', '0.050000', '0.100000', '0.250000', '0.500000', '1.000000']
trims =  ['0.100000', '0.250000', '0.500000', '1.000000']

subdirs = list()
for sigma in sigmas:
    for trim in  trims:
        subdirs.append('sigma_' + sigma + '-trim_' + trim)

for subdir in subdirs:
    alphaT = None
    betaT = None
    gammaT = None
    xT = None
    yT = None
    zT = None
    with open(subdir + '\\configuration.txt') as file:
        alphaT = float(file.readline())
        betaT = float(file.readline())
        gammaT = float(file.readline())
        xT = float(file.readline())
        yT = float(file.readline())
        zT = float(file.readline())
    
    plt.plotfile(subdir + '\\MSE.txt')
    plt.savefig(subdir + '\\MSE.png')

    plt.plotfile(subdir + '\\rotation.txt', (0, 1, 2, 3), subplots = False, delimiter = ' ', names = ['iteration', 'alpha', 'beta', 'gamma'])
    plt.savefig(subdir + '\\rotation.png')

    plt.plotfile(subdir + '\\translation.txt', (0, 1, 2, 3), subplots = False, delimiter = ' ', names = ['iteration', 'x', 'y', 'z'])
    plt.savefig(subdir + '\\tranlsation.png')