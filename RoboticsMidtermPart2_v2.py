# %% [markdown]
# # Midterm 2024
# * Copy this file to your homework workspace to have access to your other kinematic and visualization functions

# %%
# To test your setup, after defining the robot arm as described below, (but nothing else)
# you can run this file directly to make sure it is plotting the arm, obstacle, and goal 
# as expected. 

import kinematics as kin  #this is your kinematics file that you've been developing all along
from visualization import VizScene #this is the visualization file you've been using for homework
import time
import numpy as np


# Define your kinematics and an "arm" variable here using DH parameters so they
# are global variables that are available in your function below:

dh = np.array([[np.pi/2, 4, 0, np.pi/2],
               [np.pi/6, 0, 0, np.pi/2],
               [0, 4, 0, np.pi/2],
               [np.pi/6, 0, 2, np.pi/2]])
arm = kin.SerialArm(dh)


# let's also plot robot to make sure it matches what we think it should
# (this will look mostly like the pictures on part 1 if your DH parameters
# are correct)
# viz_check = VizScene()
# viz_check.add_arm(arm, joint_colors=[np.array([0.95, 0.13, 0.13, 1])]*arm.n)
# viz_check.update(qs = [[0, 0, 0, 0]])
# viz_check.hold() 

def get_obstacle_distances(q, obstacle_pos, obstacle_rad):
            distances = []
            cur_joint_position = arm.fk(q, index=3)[:3,3]
            joint_dist_to_obstacle = obstacle_pos - cur_joint_position
            distances.append(joint_dist_to_obstacle)
            
            cur_joint_position = arm.fk(q)[:3,3]
            joint_dist_to_obstacle = obstacle_pos - cur_joint_position
            distances.append(joint_dist_to_obstacle)
            return distances

def get_error(q, goal, index = arm.n):
            cur_position = arm.fk(q)[:3,3]
            e = goal - cur_position
            return e

# distances = get_obstacle_distances([0, 0, 0, 0], [0, 3, 2], 1.0)
# print (distances)
# print (np.linalg.norm(distances, axis = 1))

def compute_robot_path(q_init, goal, obst_location, obst_radius):
      # this can be similar to your IK solution for HW 6, but will require modifications
      # to make sure you avoid the obstacle as well.

      # "q_s" is an empty list that we'll use to return our solutions so they can be plotted.
      # Unlike with IK where all we are interested in is the last joint angle, here we are
      # interested in the intermediate angles as they define a path that avoids the obstacle. 
      q_s = []
      error = get_error(q_init, goal)
      count = 0
      kd = 0.09
      K_pull = np.eye(3)* .01
      K_push = np.eye(3)* .02
      q = q_init
      distances = get_obstacle_distances(q_init, obst_location, obst_radius)
      q_prev = q

      while (np.linalg.norm(error) > 1e-4 and count < 3000): # set your stopping criteria here
            if np.min((np.linalg.norm(distances, axis = 1))) < obst_radius * 1.8:
                  shortest_index = np.linalg.norm(distances, axis = 1).argmin()
                  J = arm.jacob(q, index=shortest_index + 3)[:len(goal),:]
                  J_dag = J.T @ np.linalg.inv(J @ J.T + kd**2)
                  error = (get_error(q, obst_location, index = shortest_index)) * (1/(np.min(np.linalg.norm(distances, axis = 1))) * -1)
                  q_dot = (J_dag @ K_push @ error.T) 
                  # if (q_dot.max() > 0.01):
                  #       q_dot = q_dot * 0.01/q_dot.max()
                  q = q + q_dot
            else: 
                  e = get_error(q, goal)
                  J = arm.jacob(q)[:len(goal),:]
                  J_dag = J.T @ np.linalg.inv(J @ J.T + kd**2)[:len(goal),:]
                  qdot = J_dag @ K_pull @ e
                  if (q_dot.max() > 0.01):
                        q_dot = q_dot * 0.01/q_dot.max()
                  q = q + qdot
            error = get_error(q, goal)
            count += 1
            distances = get_obstacle_distances(q, obst_location, obst_radius)
            if count > 2999: 
                  print("Max iterations reached")
            q_prev = q
            q_s.append(q)

      return q_s

if __name__ == "__main__":

      # if your function works, this code should show the goal, the obstacle, and your robot moving towards the goal.
      # Please remember that to test your function, I will change the values below to see if the algorithm still works.
      q_0 = [0, 0, 0, 0]
      goal = [0, 2, 4]
      obst_position = [0, 3, 2]
      obst_rad = 1.0

      q_ik_slns = compute_robot_path(q_0, goal, obst_position, obst_rad)
      print(q_ik_slns)


      # if you just want to check if you have your code set up and arm defined correctly, you can uncomment the next three lines 
      # and run this file using either vs code or the terminal (and running "python3 midterm_2024.py"). None of the next three 
      # lines are needed for your solution, they may just help you check your visualization before you get going. It will just 
      # display 100 different random sets of joint angles as well as the goal and obstacle.

      # import numpy as np
      # q_ik_slns = np.random.uniform(size=(100,4))
      # q_ik_slns = q_ik_slns.tolist()


      # depending on how you store q_ik_slns inside your function, you may need to change this for loop
      # definition. However if you store q as I've done above, this should work directly.
      viz = VizScene()
      viz.add_arm(arm, joint_colors=[np.array([0.95, 0.13, 0.13, 1])]*arm.n)
      viz.add_marker(goal, radius=0.1)
      viz.add_obstacle(obst_position, rad=obst_rad)
      for q in q_ik_slns:
            viz.update(qs=[q])

            # if your step in q is very small, you can shrink this time, or remove it completely to speed up your animation
            time.sleep(0)
