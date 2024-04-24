#!/usr/bin/env python3
 
import gtsam
import numpy as np
import math
import yaml
import os

import matplotlib.pyplot as plt
import gtsam.utils.plot as gtsam_plot
from gtsam.symbol_shorthand import L, X
# from graphviz import Digraph

from collections import namedtuple

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w] //no wlasnie chyba [w x y z] //maja
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q    

def update_position(x, y, theta, phi, u1, u2, L, ts):
    """
    Update the position and orientation of a car based on its speed and steering angle.
    
    Parameters:
    - x, y, theta: Current position (x, y) and orientation (theta) of the car.
    - u1: Speed of the car.
    - u2: the turning rate of the virtual front wheel.
    - phi: Steering angle of the car.
    - L: Distance between the front and rear axles of the car.
    - ts: Time increment for the update.
    
    Returns:
    - x, y, theta, phi: Updated position and orientation of the car.
    """
    # Calculate the new orientation of the car
    theta += (u1 / L) * math.tan(phi) * ts
    
    # Calculate the new position of the car
    x += u1 * math.cos(theta) * ts
    y += u1 * math.sin(theta) * ts
    phi += u2 * ts
    
    return x, y, theta, phi

import csv

filename = 'listy.txt'

list_of_lists = []

with open(filename, 'r') as file:
    reader = csv.reader(file)
    for row in reader:
        size = int(row[0])  
        current_list = [float(num) for num in row[1:] if num]  # Pomijamy pierwszy element i konwertujemy resztę
        
        if len(current_list) != size:
            print(f"Ostrzeżenie: Rozmiar listy ({len(current_list)}) nie zgadza się z oczekiwanym ({size}).")
        
        list_of_lists.append(current_list)

# for index, lst in enumerate(list_of_lists):
#     print(f"Lista {index + 1}: {lst}")

U = list_of_lists[0]
U_odom = list_of_lists[1]
U_imu = list_of_lists[2]
Q = list_of_lists[3]


graph = gtsam.NonlinearFactorGraph()

initial_estimate = gtsam.Values()

ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.03, 0.03, 0.05]))
PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))

priorMean = gtsam.Pose2(0.0, 0.0, 0.0)  # prior at origin
graph.add(gtsam.PriorFactorPose2(1, priorMean, PRIOR_NOISE))
initial_estimate.insert(1, gtsam.Pose2(0.0, 0.0, 0.0))

x = 0.0
y = 0.0
theta = 0.0
phi = 0.0

number = 1
previous_odom = gtsam.Pose2(0.0, 0.0, 0.0)

for i in range(400):
    number+=1
    u1 = U_odom[2*i]
    u2 = U_odom[(2*i)+1]
    x, y, theta, phi = update_position(x, y, theta, phi, u1, u2, 2.5, 0.1)
    print(f'x {x} y {y } th {theta}')
    pose = gtsam.Pose2(x, y, theta)
    initial_estimate.insert(number, pose)
    diff_pose = previous_odom.between(pose)
    graph.add(gtsam.BetweenFactorPose2(number-1, number, diff_pose, ODOMETRY_NOISE))

            


params = gtsam.LevenbergMarquardtParams()
# print(graph)
# print(initial_estimate)
  

optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate, params)
result = optimizer.optimize()
initial_estimate = result # przypisujemy initial z poprzedniej iteracji 

print("\nFinal Result:\n{}".format(result))
print(f"size : {result.size()}")


# marginals = gtsam.Marginals(graph, result)
plt.figure(figsize=(10,10))
for value in range(1, initial_estimate.size()):
    gtsam_plot.plot_pose2(0, result.atPose2(value), 0.5)  #, marginals.marginalCovariance(value)

# for item in betweenpose_list:
#     if result.exists(item[0]) and result.exists(item[1]):
#         x1 = result.atPose2(item[0]).x()
#         x2 = result.atPose2(item[1]).x()
#         y1 = result.atPose2(item[0]).y()
#         y2 = result.atPose2(item[1]).y()
#         plt.plot([x1,x2],[y1,y2],'b-')

plt.axis('equal')
plt.savefig('poses_gtsam.png')
# plt.close('all')

    




    
