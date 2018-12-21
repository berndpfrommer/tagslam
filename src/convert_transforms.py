import tf
import numpy as np
""" convert multicam_calib transforms to tagslam format:
example session:

ipython
import numpy as np
import convert_transforms
%load_ext autoreload
%autoreload 2
# copy T_cn_cnm1 from multicam_calibration file:
T=np.array([[0.99995273841,  0.00284628684,  0.00929621430, -0.20032164920], [-0.00285007802,  0.99999586067,  0.00039459796, -0.00109630102], [-0.00929505268, -0.00042107425,  0.99995671141,  0.00092501568], [ 0.00000000000,  0.00000000000,  0.00000000000,  1.00000000000]])
rvec,tvec = convert_transforms.multicam_to_tagslam(T)


"""

def multicam_to_tagslam(tf_matrix_4x4_cn_cnm1):
    T_w_c = np.linalg.inv(tf_matrix_4x4_cn_cnm1)
    angle, direc, point = tf.transformations.rotation_from_matrix(T_w_c)
    return angle*direc, T_w_c[0:3,3]
