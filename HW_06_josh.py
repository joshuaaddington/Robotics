import numpy as np
import sympy as sp
from visualization import VizScene 
import transforms as tr
import kinematics as kin
np.set_printoptions(precision=4, suppress=True)

alpha = ["a","b","c","d","e","f","g","h","i","j","k","l","m","n","o","p","q","r","s","t","u","v","w","x","y","z"]
dh = [[0, 0, 1, -np.pi/2.0],
      [0, 0, 1, 0]]
jt = ["r", "r"]

arm = kin.SerialArm(dh_params=dh, jt = jt)

q1a = [0, np.pi/4]
q1b = [0, np.pi/2]
q1c = [np.pi/4, np.pi/4]
q1d = [0,0]
q1e = [0,0]

q_array = [q1a, q1b, q1c, q1d, q1e]

W1a = [ [-1], [0], [0], [0], [0], [0]]
W1b = [ [-1], [0], [0], [0], [0], [0]]
W1c = [ [-1], [-1], [0], [0], [0], [0]]
W1d = [ [0], [0], [1], [0], [0], [0]]
W1e = [ [1], [0], [0], [0], [0], [0]]

W_array = [W1a, W1b, W1c, W1d, W1e]

# for i in range(5):
#     torques = arm.torques(W_array[i], q_array[i])
#     print("Tau for problem", alpha[i], "is :\n", torques)

### OUTPUT FROM PROBLEM 1 ##################
# Tau for problem  a  is : [[0.], [0.7071]]
# Tau for problem  b  is : [[0.], [1.]]
# Tau for problem  c  is : [[0.], [1.]]
# Tau for problem  d  is : [[0.], [-1.]]
# Tau for problem  e  is : [[0.], [0.]]
############################################# 

dh = [[0, 0.2, 0, -np.pi/2],
      [0, 0, 0.2, 0],
      [np.pi/2, 0, 0, np.pi/2],
      [np.pi/2, 0.4, 0, -np.pi/2],
      [0, 0, 0, np.pi/2],
      [0, 0.4, 0, 0]]
jt = ["r","r","r","r","r","r"]

arm2 = kin.SerialArm(dh_params = dh, jt = jt)

viz = VizScene()
# viz.add_arm(arm2)
# q = [0, 0, 0, 0, 0, 0]
# viz.update(qs=[q])

# tip_pos = arm2.fk(q)[:3,3]
# tip_marker = [tip_pos[0], tip_pos[1], tip_pos[2]]
# viz.add_marker(tip_marker)

target_array = np.array([
    [-0.149, 0.364, 1.03],
    [-0.171, -0.682, -0.192],
    [0.822, -0.1878, 0.533],
    [-0.336, 0.095, 0.931],
    [0.335, 0.368, 0.88]
])
method = "pinv"

for i in range(len(target_array)):
    result = arm2.ik_position(target = target_array[i], q0 = np.array([0, 0, 0, 0, 0, 0]), method = method, animate = True, K = np.array([[.05, 0, 0], [0, .05, 0], [0, 0, .05]]), max_iter = 1000, tol = 1e-6)
    if (method=="pinv"):
        print("TARGET", i+1, "RESULTS: (Pseudo-inverse method)")
    else:
        print("TARGET", i+1, "RESULTS: (Jacobian transpose method)")
    print("Resultant joint angles are: \n", result[0])
    print("Final error is : \n", result[1])
    print("counts:\n", result[2])


### PASTED CODE FROM IK_POSITION FUNCTION IN KINEMATICS.PY ###
# def get_error(q):
#             cur_position = self.fk(q)[:3,3]
#             e = target - cur_position
#             return e
        
#         error = get_error(q)

#         if animate == True:
#                     viz = VizScene()
#                     viz.update(qs=[q])
#                     viz.add_arm(self)
#                     viz.add_marker(target)
#         while np.linalg.norm(error) > tol and count < max_iter:
#             if method == 'J_T': # Jacobian transpose method
#                 e = get_error(q)
#                 J = self.jacob(q)[:len(target),:]
#                 qdot = J.T @ K @ e
#                 q = q + qdot
#                 error = get_error(q)
#                 count += 1
#                 if animate == True:
#                     viz.update(qs=[q])
#             elif method == "pinv": # Pseudo-inverse method
#                 e = get_error(q)
#                 J = self.jacob(q)
#                 J_dag = J.T @ np.linalg.inv(J @ J.T + kd**2)
#                 qdot = J_dag @ e
#                 q = q + qdot
#                 error = get_error(q)
#                 count += 1
#                 if animate == True:
#                     viz.update(qs=[q])
############  END OF PASTED CODE  ############################


############# OUTPUT FROM PROBLEM 3 ############################
#
# TARGET 1 RESULTS: (Pseudo-inverse method)
# Resultant joint angles are: 
#  [ 1.2773 -0.822  -0.6527 -0.3929  0.7356  0.    ]
# Final error is : 
#  [ 0. -0.  0.]
# counts:
#  8

# TARGET 2 RESULTS: (Pseudo-inverse method)
# Resultant joint angles are: 
#  [ -8.2371  12.8248 -17.8344   1.3233  13.953   -0.    ]
# Final error is :
#  [-0. -0. -0.]
# counts:
#  5

# TARGET 3 RESULTS: (Pseudo-inverse method)
# Resultant joint angles are:
#  [-6.1511 -0.2038 -0.384   0.2844  5.4093 -0.    ]
# Final error is :
#  [ 0. -0.  0.]
# counts:
#  7

# TARGET 4 RESULTS: (Pseudo-inverse method)
# Resultant joint angles are:
#  [ 1.7985 -0.378  -1.2451 -0.0551  0.8724 -0.    ]
# Final error is :
#  [-0.  0.  0.]
# counts:
#  6

# TARGET 5 RESULTS: (Pseudo-inverse method)
# Resultant joint angles are:
#  [ 0.3149 -0.3571 -1.1884 -0.7506  1.0001 -0.    ]
# Final error is :
#  [0. 0. 0.]
# counts:
#  6
################################################################
################################################################
# 
# TARGET 1 RESULTS: (Jacobian transpose method)
# Resultant joint angles are:
#  [ 1.1446 -1.0368 -0.3331 -0.1163  0.8031  0.    ]
# Final error is :
#  [-0.0002  0.0002  0.0003]
# counts:
#  1000

# TARGET 2 RESULTS: (Jacobian transpose method)
# Resultant joint angles are:
#  [-1.2451  0.2711  0.4     0.0449 -1.2588  0.    ]
# Final error is :
#  [0.     0.0001 0.    ]
# counts:
#  240

# TARGET 3 RESULTS: (Jacobian transpose method)
# Resultant joint angles are:
#  [ 0.1092  0.0533 -0.5485 -0.0676 -0.7648  0.    ]
# Final error is :
#  [-0.0001  0.     -0.    ]
# counts:
#  682

# TARGET 4 RESULTS: (Jacobian transpose method)
# Resultant joint angles are:
#  [ 0.1546 -0.6589 -1.5556  0.1033  0.3748  0.    ]
# Final error is :
#  [ 0.0001 -0.     -0.0001]
# counts:
#  735

# TARGET 5 RESULTS: (Jacobian transpose method)
# Resultant joint angles are:
#  [ 0.1575 -0.3153 -0.8756  0.1433  0.9034 -0.    ]
# Final error is :
#  [-0.     -0.0001 -0.0001]
# counts:
#  471

################################################################

fk_test1_J_T = arm2.fk([1.1446, -1.0368, -0.3331, -0.1163,  0.8031,  0.])[:3,3]
fk_test1_pinv = arm2.fk([1.2773, -0.822, -0.6527, -0.3929,  0.7356,  0.])[:3,3]
fk_test2_J_T = arm2.fk([-1.2451, 0.2711, 0.4, 0.0449, -1.2588, 0.])[:3,3]
fk_test2_pinv = arm2.fk([-8.2371, 12.8248, -17.8344, 1.3233, 13.953, 0.])[:3,3]
fk_test3_J_T = arm2.fk([0.1092, 0.0533, -0.5485, -0.0676, -0.7648, 0.])[:3,3]
fk_test3_pinv = arm2.fk([-6.1511, -0.2038, -0.384, 0.2844, 5.4093, 0.])[:3,3]
fk_test4_J_T = arm2.fk([0.1546, -0.6589, -1.5556, 0.1033, 0.3748, 0.])[:3,3]
fk_test4_pinv = arm2.fk([1.7985, -0.378, -1.2451, -0.0551, 0.8724, 0.])[:3,3]
fk_test5_J_T = arm2.fk([0.1575, -0.3153, -0.8756, 0.1433, 0.9034, 0.])[:3,3]
fk_test5_pinv = arm2.fk([0.3149, -0.3571, -1.1884, -0.7506, 1.0001, 0.])[:3,3]

print("Position for problem 1 using Jacobian transpose method and then Pseudo-inverse method are: \n", fk_test1_J_T,"\n", fk_test1_pinv)
print("Position for problem 2 using Jacobian transpose method and then Pseudo-inverse method are: \n", fk_test2_J_T,"\n", fk_test2_pinv)
print("Position for problem 3 using Jacobian transpose method and then Pseudo-inverse method are: \n", fk_test3_J_T,"\n", fk_test3_pinv)
print("Position for problem 4 using Jacobian transpose method and then Pseudo-inverse method are: \n", fk_test4_J_T,"\n", fk_test4_pinv)
print("Position for problem 5 using Jacobian transpose method and then Pseudo-inverse method are: \n", fk_test5_J_T,"\n", fk_test5_pinv)

################# OUTPUT FROM POSITION TESTS #####################
# Position for problem 1 using Jacobian transpose method and then Pseudo-inverse method are: 
#  [-0.1488  0.3638  1.0297]
#  [-0.149  0.364  1.03 ]
# Position for problem 2 using Jacobian transpose method and then Pseudo-inverse method are:
#  [-0.1711 -0.6821 -0.192 ]
#  [-0.171 -0.682 -0.192]
# Position for problem 3 using Jacobian transpose method and then Pseudo-inverse method are:
#  [ 0.8221 -0.1879  0.533 ]
#  [ 0.822  -0.1878  0.533 ]
# Position for problem 4 using Jacobian transpose method and then Pseudo-inverse method are:
#  [-0.3361  0.095   0.9311]
#  [-0.336  0.095  0.931]
# Position for problem 5 using Jacobian transpose method and then Pseudo-inverse method are:
#  [0.335  0.3681 0.8801]
#  [0.335 0.368 0.88 ]
#
# The positions are very close, but the pinv method was consistently faster and thus more accurate.
#
##################################################################

# viz = VizScene()
# pos = [-0.149,  0.364,  1.03]
# q = [0, 0, 0, 0, 0, 0]
# q_JT = [0.4827, -0.9903, -0.7087, -0.0231,  0.9704, -0.]
# q_pinv = [1.2773, -0.822,  -0.6527, -0.3929,  0.7356,  0.]
# viz.add_arm(arm2) #the base configuration
# viz.add_arm(arm2, joint_colors=[[1, 51.0/255.0, 51.0/255.0, 1]]*arm2.n) #The first arm solution (pinv solution)
# viz.add_arm(arm2, joint_colors=[[1, 51.0/255.0, 1, 1]]*arm2.n) #The second arm solution (JT solution)
# viz.add_marker(pos) #Add the marker for desired position
# viz.update(qs=[q, q_pinv, q_JT]) #Update the qs in the same order as you added the arm (original, pinv, JT in that order for this code)
# viz.hold() #Hold the visualization until you close it