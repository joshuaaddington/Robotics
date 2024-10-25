import numpy as np
import kinematics as kin
from visualization import VizScene
import time

q_0 = [0, 0, 0, 0]
goal = [0, 2, 4]
obst_position = [0, 3, 2]
obst_rad = 1.0
q_s = []
count = 0
kd = 0.001
K = np.eye(3)* .1
q = q_0

dh = np.array([[np.pi/2, 4, 0, np.pi/2],
               [np.pi/6, 0, 0, np.pi/2],
               [0, 4, 0, np.pi/2],
               [np.pi/6, 0, 2, np.pi/2]])
arm = kin.SerialArm(dh)

def get_obstacle_distances(q, obstacle_pos, obstacle_rad):
            distances = []
            for i in range(arm.n):
                cur_joint_position = arm.fk(q, index=i)[:3,3]
                joint_dist_to_obstacle = obstacle_pos - cur_joint_position
                distances.append(joint_dist_to_obstacle)
            return distances


distances = get_obstacle_distances([0, 0, 0, 0], [0, 3, 2], 1.0)
print (distances)
print ((np.linalg.norm(distances, axis = 1)))

if np.min((np.linalg.norm(distances, axis = 1))) < (obst_rad * 1.2):
        print ("Obstacle is too close to the robot. Please move it further away.")
        shortest_index = np.linalg.norm(distances, axis = 1).argmin()
        print(shortest_index)
        # print (min(np.linalg.norm(distances, axis = 1)))