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

def make_wph(l=1.0, h=2.0):
    
    wph = np.matrix([[-l,  h, 0, 1],
                     [ l,  h, 0, 1],
                     [ l, -h, 0, 1],
                     [-l, -h, 0, 1]])
    return wph


def plot_image_points(ip1h, ip2h=None, ip3h=None):
    plt.scatter(np.squeeze(np.array(ip1h[:,0])), np.squeeze(np.array(ip1h[:,1])),c='blue')
    if not (ip2h is None):
        plt.scatter(np.squeeze(np.array(ip2h[:,0])), np.squeeze(np.array(ip2h[:,1])),c='red')
    if not (ip3h is None):
        plt.scatter(np.squeeze(np.array(ip3h[:,0])), np.squeeze(np.array(ip3h[:,1])),c='green')
    plt.show()

def make_tf(beta=0, alpha=0, gamma=0, d=3.0):
    T = np.matrix(rotation_matrix(gamma, (0,0,1))) * \
        np.matrix(rotation_matrix(beta,  (0,1,0))) * \
        np.matrix(rotation_matrix(alpha, (1,0,0)))
    T[0:3,3] = np.matrix([[0, 0, d]]).T
    return T

def project_points(wph_cam):
    return (wph_cam/wph_cam[:,2])[:,0:3]

def V_from_v(v):
    V = np.zeros((v.shape[0], v.shape[1], v.shape[1]))
    for i in range(0,v.shape[0]):
        V[i,:,:] = v[i,:].T*v[i,:]/(v[i,:]*v[i,:].T)
    return V

def make_homogenous(x):
    if len(x.shape) == 1:
        x = np.expand_dims(x, axis=0)
    xh = np.hstack((x, np.ones((x.shape[0], 1))))
    return xh

def rotate_to_z(translat):
    z = np.array([0, 0, 1])
    t = np.squeeze(np.asarray(translat))
    t_norm = np.linalg.norm(t)
    txz = np.cross(t, np.asarray(z))
    sin_a_t_norm = np.linalg.norm(txz)
    if abs(sin_a_t_norm) < 1e-8:
        return np.matrix(np.eye(4))
    sin_a = sin_a_t_norm / t_norm
    n = txz / sin_a_t_norm
    print('angle: ', math.asin(sin_a), ' ', np.linalg.norm(n))
    R = np.matrix(rotation_matrix(math.asin(sin_a), n))
    return R

def decompose_R1Tilde(R1T):
    # compute R1T = Rz * Ry * Rx
    e  = euler_from_matrix(R1T, 'rzyx')
    Rz = np.matrix(rotation_matrix(e[0], (0,0,1)))
    Ry = np.matrix(rotation_matrix(e[1], (0,1,0)))
    Rx = np.matrix(rotation_matrix(e[2], (1,0,0)))
    # RzTilde.inverse() = Rx
    return Rz, Ry, Rx, e[0], e[1]

def eos(wph, iph, T):
    V = V_from_v(iph)
    wp_trans = (T * wph.T)[0:3,:]
    I = np.matrix(np.eye(3))
    E = 0
    for i in range(0,wph.shape[0]):
        err = (I - V[i,:,:])*wp_trans[:,i]
        E = E + err.T*err
    return E[0,0]

def compute_G(Vt):
    n   = Vt.shape[0]
    ImVs = np.matrix(np.zeros((3,3)))
    for i in range(0, n):
        ImV = np.matrix(np.eye(3)) - Vt[i,:,:]
        ImVs = ImVs + ImV.T * ImV
    return np.linalg.inv(ImVs)

def make_K(pt):
    p = np.squeeze(np.array(pt))
    return np.matrix([[p[0], 2*p[2],  -p[0]],
                      [p[1], 0,        p[1]],
                      [p[2], -2*p[0], -p[2]]])

def compute_FTF(Vt, Rz, pt):
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

def compute_polynomial(F):
    f = np.asarray([F[0, 0],  F[0, 1] + F[1, 0],
                    F[0, 2] + F[1, 1] + F[2, 0],
                    F[1, 2] + F[2, 1],  F[2, 2]])
    # compute first deriv polynomial
    # arrange the coefficients THE OTHER WAY ROUND: highest degree comes first!
    g = np.asarray([-f[3], 4*f[4]-2*f[2], 3*(f[3]-f[1]), 2*f[2]-4*f[0], f[1]])
    # arrange the coefficients THE OTHER WAY ROUND: highest degree comes first!
    h = np.asarray([2*f[3],            # 5
                    6 *f[2] - 12*f[4], # 4
                    12*f[1] - 16*f[3], # 3
                    20*f[0] - 16*f[2] + 12*f[4], # 2
                    -12*f[1] + 6*f[3], # 1
                    -4*f[0] +  2*f[2]])
    return f, g, h

def compute_optimal_t(Vt, Rz, beta, pt):
    #G = compute_G(Vt)

    t  = np.matrix(np.zeros((3,1)))
    Ry = np.matrix(rotation_matrix(beta, (0,1,0)))
    print 'pt=\n', pt
    print 'using beta: ', beta
    G = np.matrix(np.zeros((3,3)))
    for j in range(0, Vt.shape[0]):
        ImV = np.matrix(np.eye(3)) - Vt[j]
        t = t + ImV * ImV * Rz[0:3,0:3] * Ry[0:3,0:3] * pt[j,0:3].T
        G = G + ImV * ImV
    return -np.linalg.inv(G) * t
    
def find_optimal_betas(wp, ip, T1):
    n   = wp.shape[0]   # number of points
    wph = make_homogenous(wp)
    iph = make_homogenous(ip)
    # compute rotation matrix Rt for use in equation (5)
    Rt  = rotate_to_z(np.squeeze(T1[0:3,3]))
    # R1T = R1Tilde, from equation 5
    R1T = Rt * T1
    Rz, Ry, Rx, gamma_1, beta_1 = decompose_R1Tilde(R1T)
    vt  = (Rt[0:3,0:3] * iph.T).T  # v_tilde, equation (5)
    pt  = (Rx * wph.T).T # p_tilde, between equation 7/8
    Vt  = V_from_v(vt)   # V_tilde_i
    FTF = compute_FTF(Vt, Rz[0:3, 0:3], pt)
    f, g, h  = compute_polynomial(FTF)
    #
    # compute optimal t for test beta
    #
    if False:
        beta_test = 0.0
        t_opt = compute_optimal_t(Vt, Rz, beta_test, pt)
        print 'optimal t for  beta=', beta_test, ': ', t_opt
        Ry_test = np.matrix(rotation_matrix(beta_test, (0,1,0)))
        Tt  = Rz * Ry_test
        Tt[0:3,3] = t_opt
        print 'E_os with t optimal for beta=', beta_test, ': ', eos(pt, vt, Tt)
        Tt[0:3,3] = T1[0:3,3]
        print 'E_os with T from t1  for beta=', beta_test, ': ', eos(pt, vt, Tt)
        print 'E_os with original t1: ', eos(pt, vt, T1)

    
    return f, g, h

def eos_from_poly(beta, f):
    x = np.tan(beta/2.0)
    y = f[0] + x * f[1] + x*x * f[2] + x*x*x*f[3] + x*x*x*x*f[4]
    y = y / ((1.0 + x*x)*(1.0 + x*x))
    return y

def eos_deriv(beta, g):
    x = np.tan(beta/2.0)
    y = g[4] + x * g[3] + x*x * g[2] + x*x*x*g[1] + x*x*x*x*g[0]
#    y = y / ((1.0 + x*x)*(1.0 + x*x)*(1.0 + x*x))
    return y

def eos_from_full(x, pt, vt, Rz, Tt):
    y = np.zeros(x.shape[0])
    for i in range(0, x.shape[0]):
        Ry = np.matrix(rotation_matrix(x[i], (0,1,0)))
        R  = Rz * Ry
        R[0:3,3] = Tt[0:3,3]
        y[i] = eos(pt, vt, R)
    return y
    
def main(wp, ip, T1):
    wph = make_homogenous(wp)
    iph = make_homogenous(ip)
    # compute rotation matrix Rt for use in equation (5)
    Rt  = rotate_to_z(np.squeeze(T1[0:3,3]))
    # R1T = R1Tilde, from equation 5
    R1T = Rt * T1
    Rz, Ry, Rx, gamma_1, beta_1 = decompose_R1Tilde(R1T)
    print 'beta_1: ', beta_1
    vt  = (Rt[0:3,0:3] * iph.T).T  # v_tilde, equation (5)
    pt  = (Rx * wph.T).T # p_tilde, between equation 7/8
    # Tt is the combination of Rz*Ry and t_tilde
    Tt  = Rz * Ry
    Tt[0:3,3] = np.squeeze(R1T[0:3,3]).T
    # all 3 of those should be identical
    print 'E_os original:      ', eos(wph, iph, T1)
    print 'E_os rotated once:  ', eos(wph, vt, R1T)
    print 'E_os rotated twice: ', eos(pt, vt, Tt)
    Ry0 = np.matrix(rotation_matrix(0.0, (0,1,0)))
    Tt0  = Rz
    Tt0[0:3,3] = np.squeeze(R1T[0:3,3]).T
    print 'E_os for beta=0: ', eos(pt, vt, Tt0)

    f, g, h = find_optimal_betas(wp, ip, T1)
    print 'E_os from polynomial: ', eos_from_poly(beta_1, f)
    print 'coeff: ', f
    print 'deriv: ', g
    print '2nd deriv: ', h
    print 'roots: ', np.roots(g)
    print 'roots conv: ', 2*np.arctan(np.roots(g))
    print 'second deriv: ', np.polyval(h, np.roots(g))

    beta = np.linspace(-3.5, 1.0, 50)
    plt.plot(beta, eos_from_poly(beta, f), 'b')
#    plt.plot(beta, eos_deriv(beta, g), 'k')
#    plt.plot(beta, eos_from_full(beta, pt, vt, Rz, Tt),'r')
    plt.show()
