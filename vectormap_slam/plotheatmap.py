from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
import math
from copy import copy
from mpl_toolkits.mplot3d import Axes3D


GridSize = 51
MaxOffset = 2.0


if (__name__ == '__main__') :
    
    fig = plt.figure()
    ax = fig.add_subplot (111, projection='3d')
    
    step = MaxOffset / math.floor(GridSize/2)
    X = np.arange(-MaxOffset, MaxOffset+step, step)
    Y = copy(X)
    Z = np.loadtxt ("/tmp/heatmap.csv", delimiter=",")
    Xm, Ym = np.meshgrid(X, Y)

    ax.plot_surface (Xm, Ym, Z)
    
    plt.show()
    pass