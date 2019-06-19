import tf
import numpy as np
""" convert multicam_calib transforms to tagslam format:
example session:

ipython
import numpy as np
from convert_transforms import multicam_to_tagslam
%load_ext autoreload
%autoreload 2
# copy T_cn_cnm1 from multicam_calibration file:
T=np.array([[0.99995273841,  0.00284628684,  0.00929621430, -0.20032164920], [-0.00285007802,  0.99999586067,  0.00039459796, -0.00109630102], [-0.00929505268, -0.00042107425,  0.99995671141,  0.00092501568], [ 0.00000000000,  0.00000000000,  0.00000000000,  1.00000000000]])
rvec,tvec = multicam_to_tagslam(T)


"""

def mat_to_rvec_tvec(T):
    angle, direc, point = tf.transformations.rotation_from_matrix(T)
    return angle*direc, T[0:3,3]

def multicam_to_tagslam(tf_matrix_4x4_cn_cnm1):
    T_w_c = np.linalg.inv(tf_matrix_4x4_cn_cnm1)
    return mat_to_rvec_tvec(T_w_c)

def rvec_tvec_to_mat(rvec, tvec):
    l = np.linalg.norm(rvec)
    n = rvec/l if l > 1e-8 else np.array([1.0, 0.0, 0.0])
    T = tf.transformations.rotation_matrix(l, n)
    T[0:3, 3] = tvec
    return T

def as_yaml(rvec, tvec):
    print "    position:"
    print "      x: %12.8f" % tvec[0]
    print "      y: %12.8f" % tvec[1]
    print "      z: %12.8f" % tvec[2]
    print "    rotation:"
    print "      x:  %11.8f" % rvec[0]
    print "      y:  %11.8f" % rvec[1]
    print "      z:  %11.8f" % rvec[2]
