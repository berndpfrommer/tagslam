import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def plot_cylinder(tf, ax, radius, length, radial_ngrid, length_ngrid, color):
    x    = np.linspace(-1,      1, radial_ngrid)
    z    = np.linspace( 0, length, length_ngrid)
    X, Z = np.meshgrid(x, z)
    Y    = np.sqrt(1.0 - X**2) * radius
    X    = X * radius
    Z    = Z 

    rstride = 50
    cstride = 50
    # create homogeneous coordinates and apply transform
    p1 = np.matmul(tf, np.vstack([X.flatten()[None,:], Y.flatten()[None,:], Z.flatten()[None,:], np.ones_like(X.flatten()[None,:])])).T
    # restore original shape
    Xp1 = p1[:,0].reshape(X.shape)
    Yp1 = p1[:,1].reshape(Y.shape)
    Zp1 = p1[:,2].reshape(Z.shape)
    # same for the other half
    p2 = np.matmul(tf, np.vstack([X.flatten()[None,:], -Y.flatten()[None,:], Z.flatten()[None,:], np.ones_like(X.flatten()[None,:])])).T
    Xp2 = p2[:,0].reshape(X.shape)
    Yp2 = p2[:,1].reshape(Y.shape)
    Zp2 = p2[:,2].reshape(Z.shape)
    # plot one half of cylinder
    ax.plot_surface(Xp1, Yp1, Zp1, rstride=rstride, cstride=cstride, color=color)
    # plot the other half
    ax.plot_surface(Xp2, Yp2, Zp2, rstride=rstride, cstride=cstride, color=color)



def visualize_transform(tf_matrix_4x4):
    # make plots
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    l = 1.0
    Rx = np.array([[1, 0, 0, 0], [ 0, 0, 1, 0],[0,-1,0,0],[0,0,0,1]])
    Ry = np.array([[0, 0, 1, 0], [ 0, 1, 0, 0],[-1,0,0,0],[0,0,0,1]])
    R0 = np.array([[1, 0, 0, 0], [ 0, 1, 0, 0],[0,0,1,0],[0,0,0,1]])
    plot_cylinder(np.matmul(tf_matrix_4x4, Ry), ax, radius=0.1*l, length=l, radial_ngrid=100, length_ngrid=50,color='r')
    plot_cylinder(np.matmul(tf_matrix_4x4, Rx), ax, radius=0.1*l, length=l, radial_ngrid=100, length_ngrid=50,color='g')
    plot_cylinder(np.matmul(tf_matrix_4x4, R0), ax, radius=0.1*l, length=l, radial_ngrid=100, length_ngrid=50,color='b')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()
