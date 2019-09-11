#!/usr/bin/python
#
# 2019 Bernd Pfrommer
#
# This is test code written to implement
#
# "Robust Pose Estimation from a Planar Target"
# by Gerald Schweighofer and Alex Pinz

# I believe I found some errors in their paper, see documentation
# in rpp.cpp, the C++ source code.

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tf.transformations import *
import math

"""
import numpy as np
import matplotlib.pyplot as plt
import rpp
%load_ext autoreload
%autoreload 2
plt.ion()  # switch interactive mode on
"""

def make_wph(w=1.0, h=2.0):
    """ make homogeneous world points for rectangular tag of
        width w and length l """
    wph = np.matrix([[-w,  h, 0, 1],
                     [ w,  h, 0, 1],
                     [ w, -h, 0, 1],
                     [-w, -h, 0, 1]])
    return wph

def plot_image_points(ip1h, ip2h=None, ip3h=None):
    """ plot multiple homogeneous image points as scatter plot"""
    plt.scatter(np.squeeze(np.array(ip1h[:,0])),
                np.squeeze(np.array(ip1h[:,1])),c='blue')
    if not (ip2h is None):
        plt.scatter(np.squeeze(np.array(ip2h[:,0])),
                    np.squeeze(np.array(ip2h[:,1])),c='red')
    if not (ip3h is None):
        plt.scatter(np.squeeze(np.array(ip3h[:,0])),
                    np.squeeze(np.array(ip3h[:,1])),c='green')
    plt.gca().axis('equal')
    plt.show()

def make_tf(beta=0, alpha=0, gamma=0, d=3.0):
    """ make transform T = R_z(gamma) * R_y(beta) * R_x(alpha) """
    T = np.matrix(rotation_matrix(gamma, (0,0,1))) * \
        np.matrix(rotation_matrix(beta,  (0,1,0))) * \
        np.matrix(rotation_matrix(alpha, (1,0,0)))
    T[0:3,3] = np.matrix([[0, 0, d]]).T
    return T

def project_points(wph_cam):
    """ project homogeneous world points in camera coord to image points """
    return (wph_cam/wph_cam[:,2])[:,0:3]

def V_from_v(v):
    """ make V matrices from rows of v, see paper """
    V = np.zeros((v.shape[0], v.shape[1], v.shape[1]))
    for i in range(0,v.shape[0]):
        V[i,:,:] = v[i,:].T*v[i,:]/(v[i,:]*v[i,:].T)
    return V

def make_homogeneous(x):
    """ make homogeneous coordinates by appending 1 """
    if len(x.shape) == 1:
        x = np.expand_dims(x, axis=0)
    xh = np.hstack((x, np.ones((x.shape[0], 1))))
    return xh

def rotate_to_z(translat):
    """ compute rotation to rotate optical axis to point to tag center"""
    z = np.array([0, 0, 1])
    t = np.squeeze(np.asarray(translat))
    t_norm = np.linalg.norm(t)
    txz = np.cross(t, np.asarray(z))
    sin_a_t_norm = np.linalg.norm(txz)
    if abs(sin_a_t_norm) < 1e-8:
        return np.matrix(np.eye(4))
    sin_a = sin_a_t_norm / t_norm
    n = txz / sin_a_t_norm
    R = np.matrix(rotation_matrix(math.asin(sin_a), n))
    return R

def decompose_R1Tilde(R1T):
    """ decompose \tilde{R1}, see paper """
    # compute R1T = Rz * Ry * Rz0
    e  = euler_from_matrix(R1T, 'rzyz')
    Rz  = np.matrix(rotation_matrix(e[0], (0,0,1)))
    Ry  = np.matrix(rotation_matrix(e[1], (0,1,0)))
    Rz0 = np.matrix(rotation_matrix(e[2], (0,0,1)))
    # RzTilde.inverse() = Rz0
    return Rz, Ry, Rz0, e[0], e[1]

def eos(wph, iph, T):
    """ computes error function E_os for given world, img points,
        and transform"""
    V = V_from_v(iph)
    wp_trans = (T * wph.T)[0:3,:]
    I = np.matrix(np.eye(3))
    E = 0
    for i in range(0,wph.shape[0]):
        err = (I - V[i,:,:])*wp_trans[:,i]
        E = E + err.T*err
    return E[0,0]

def compute_G(Vt):
    """ computes constant G from the paper (with fixes, not original!)"""
    n   = Vt.shape[0]
    ImVs = np.matrix(np.zeros((3,3)))
    for i in range(0, n):
        ImV = np.matrix(np.eye(3)) - Vt[i,:,:]
        ImVs = ImVs + ImV.T * ImV
    return np.linalg.inv(ImVs)

def make_K(pt):
    """ The matrix K matrix is defined as
          K*[1, beta_t, beta_t^2]^T = R_y(beta_t)*p"""
    p = np.squeeze(np.array(pt))
    return np.matrix([[p[0], 2*p[2],  -p[0]],
                      [p[1], 0,        p[1]],
                      [p[2], -2*p[0], -p[2]]])

def compute_FTF(Vt, Rz, pt):
    """ computes F^T * F where
        F^T * F = sum_i F_i^T * F
    and
        F_i = (I-V_i)*Rz(gamma)*K_i -(I-Vi)*G*[sum_j(I-V_j)^2 Rz(gamma) K_i]"""
    n   = pt.shape[0]
    G   = compute_G(Vt)

    C   = np.zeros((n, 3, 3))
    Csum = np.matrix(np.zeros((3, 3)))
    for i in range(0, Vt.shape[0]):
        ImVi = np.matrix(np.eye(3)) - Vt[i,:,:]
        C[i,:,:] = ImVi * np.matrix(Rz) * make_K(pt[i,:])
        Csum = Csum + ImVi.T * C[i,:,:]

    FTF = np.matrix(np.zeros((3,3)))
    for i in range(0, Vt.shape[0]):
        ImVi = np.matrix(np.eye(3)) - Vt[i,:,:]
        Fi = np.matrix(C[i,:,:]) - ImVi * G * Csum
        FTF = FTF + Fi.T * Fi
    return FTF

def compute_polynomial(FTF):
    """
    E_os(beta_t) = (1+beta_t^2)^{-2} * mu^T * (F^T * F) * mu
    where mu = [1, beta_t, beta_t^2]^T
    
    now express E_os(beta_t) and derivates as polynomials in beta_t
    
    E_os   = (1+beta^2)^{-2} * (sum_{i=0^n} f[n-i] beta^i)
    E_os'  = (1+beta^2)^{-3} * (sum_{i=0^n} g[n-i] beta^i)
    E_os'' = (1+beta^2)^{-4} * (sum_{i=0^n} h[n-i] beta^i)

    NOTE: ordering is opposite to the c++ code!!!
          i.e. f[0] is x^4 order in python!!!
    """
    # zeroth order deriv poly
    f = np.asarray([FTF[2, 2],
                    FTF[1, 2] + FTF[2, 1],
                    FTF[0, 2] + FTF[1, 1] + FTF[2, 0],
                    FTF[0, 1] + FTF[1, 0],
                    FTF[0, 0]])
    # first deriv polynomial
    g = np.asarray([-f[1], 4*f[0]-2*f[2], 3*(f[1]-f[3]), 2*f[2]-4*f[4], f[3]])

    # second deriv polynomial
    h = np.asarray([2*f[1],            # 5
                    6 *f[2] - 12*f[0], # 4
                    12*f[3] - 16*f[1], # 3
                    20*f[4] - 16*f[2] + 12*f[0], # 2
                    -12*f[3] + 6*f[1], # 1
                    -4*f[4] +  2*f[2]])
    return f, g, h

def compute_optimal_t(Vt, Rz, beta, pt):
    """
    Computes optimal translation \tilde{t} for given beta.
    only used for testing
    """
    t  = np.matrix(np.zeros((3,1)))
    Ry = np.matrix(rotation_matrix(beta, (0,1,0)))
    G = np.matrix(np.zeros((3,3)))
    for j in range(0, Vt.shape[0]):
        ImV = np.matrix(np.eye(3)) - Vt[j]
        t = t + ImV * ImV * Rz[0:3,0:3] * Ry[0:3,0:3] * pt[j,0:3].T
        G = G + ImV * ImV
    return -np.linalg.inv(G) * t
    
def find_polynomials(wp, ip, T1):
    """
    Computes the polynomials f,g,h with 0th, 1st, and 2nd deriv of E_os:

    E_os   = (1+beta^2)^{-2} * (sum_{i=0^n} f[i] beta^i)
    E_os'  = (1+beta^2)^{-3} * (sum_{i=0^n} g[i] beta^i)
    E_os'' = (1+beta^2)^{-4} * (sum_{i=0^n} h[i] beta^i)
    """

    n   = wp.shape[0]   # number of points
    wph = make_homogeneous(wp)
    iph = make_homogeneous(ip)
    # compute rotation matrix Rt for use in equation (5)
    Rt  = rotate_to_z(np.squeeze(T1[0:3,3]))
    # R1T = R1Tilde, from equation 5
    R1T = Rt * T1
    Rz, Ry, Rz0, gamma_1, beta_1 = decompose_R1Tilde(R1T)
    vt  = (Rt[0:3,0:3] * iph.T).T  # v_tilde, equation (5)
    pt  = (Rz0 * wph.T).T # p_tilde, between equation 7/8
    Vt  = V_from_v(vt)   # V_tilde_i
    FTF = compute_FTF(Vt, Rz[0:3, 0:3], pt)
    f, g, h  = compute_polynomial(FTF)
    
    return f, g, h

def eos_from_poly(beta, f):
    """ compute E_os(beta)  from polynomial expression"""
    x = np.tan(beta/2.0)
    y = np.polyval(f, x)
    y = y / ((1.0 + x*x)*(1.0 + x*x))
    return y

def eos_deriv(beta, g):
    """ compute d E_os(beta)/d beta from polynomial expression"""
    x = np.tan(beta/2.0)
    y = g[4] + x * g[3] + x*x * g[2] + x*x*x*g[1] + x*x*x*x*g[0]
    y = y / ((1.0 + x*x)*(1.0 + x*x)*(1.0 + x*x))
    return y

def eos_from_full(x, pt, vt, Rz, Tt):
    """
    computes E_os(beta), not using polynomial, but also
    *without* optimizing t
    """
    y = np.zeros(x.shape[0])
    for i in range(0, x.shape[0]):
        Ry = np.matrix(rotation_matrix(x[i], (0,1,0)))
        R  = Rz * Ry
        R[0:3,3] = Tt[0:3,3]
        y[i] = eos(pt, vt, R)
    return y

def find_minima(f, g, h):
    """
    finds roots of g and then the real minima via h.
    returns E_os for all minima found
    """
    roots = np.roots(g)
    mins = []; E_os = [];
    for r in roots:
        if np.imag(r) < 1e-8 and np.polyval(h, r) > 0: # real, minimum
            beta_t = np.real(r)
            E = eos_from_poly(2.0*math.atan(beta_t), f)
            E_os.append(E)
            mins.append(2.0*math.atan(beta_t))
    return mins, E_os

def main(wp, ip, T1):
    """
    main driver routine that computes everything
    """
    wph = make_homogeneous(wp)
    iph = make_homogeneous(ip)
    # compute rotation matrix Rt for use in equation (5)
    Rt  = rotate_to_z(np.squeeze(T1[0:3,3]))
    # R1T = R1Tilde, from equation 5
    R1T = Rt * T1
    Rz, Ry, Rz0, gamma_1, beta_1 = decompose_R1Tilde(R1T)
    print 'beta_1: ', beta_1
    vt  = (Rt[0:3,0:3] * iph.T).T  # v_tilde, equation (5)
    pt  = (Rz0 * wph.T).T # p_tilde, between equation 7/8
    # Tt is the combination of Rz*Ry and t_tilde
    Tt  = Rz * Ry
    Tt[0:3,3] = np.squeeze(R1T[0:3,3]).T
    # all 3 of those should be identical
    print 'E_os original:                  ', eos(wph, iph, T1)
#    print 'E_os rotated once:  ', eos(wph, vt, R1T)
#    print 'E_os rotated twice: ', eos(pt, vt, Tt)

    f, g, h = find_polynomials(wp, ip, T1)
    betas, E_os = find_minima(f, g, h)
    print 'E_os from polynomial(beta_1):   ', eos_from_poly(beta_1, f)
    print 'E_os from polynomial(beta_min): ', eos_from_poly(betas[1], f)
#    print 'roots conv: ', 2*np.arctan(np.roots(g))
#    print 'second deriv: ', np.polyval(h, np.roots(g))
    print '--------- minima found -----------'
    for i in range(0, len(betas)):
        print 'E_os(', betas[i], ') = ', E_os[i]
    print 'ratio min/max: ', min(E_os)/max(E_os)
        
    beta = np.linspace(-math.pi, math.pi, 50)
    plt.plot(beta, eos_from_poly(beta, f), 'b')
#    plt.plot(beta, eos_deriv(beta, g), 'k')
#    plt.plot(beta, eos_from_full(beta, pt, vt, Rz, Tt),'r')
#    plt.show()
    return E_os, betas
