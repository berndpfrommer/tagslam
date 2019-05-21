import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

"""
import numpy as np
import matplotlib.pyplot as plt
from visualize import visualize_transform
%load_ext autoreload
%autoreload 2
plt.ion()  # switch interactive mode on
"""

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

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



def visualize_transform(tf_matrix_4x4, wp=np.array([0,0,0]), sz=20.0):
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
    wpa = np.array(wp)
    ax.scatter(wpa[0,0], wpa[0,1], wpa[0,2], s=sz, c='r')
    ax.scatter(wpa[1,0], wpa[1,1], wpa[1,2], s=sz, c='g')
    ax.scatter(wpa[2,0], wpa[2,1], wpa[2,2], s=sz, c='b')
    ax.scatter(wpa[3,0], wpa[3,1], wpa[3,2], s=sz, c='k')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.axis('equal')
    plt.gca().set_aspect('equal', adjustable='box')
    set_axes_equal(ax)
    
    plt.show()
