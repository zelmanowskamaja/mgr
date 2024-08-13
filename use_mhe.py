import csv
import math
import numpy as np 

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
Q_noise = list_of_lists[3]
Q = list_of_lists[4]

P = U + U_odom + U_imu + Q_noise

import sys
sys.path.insert(1, '/home/maja/Documents/mgr/mhe_optimizers/mhe_optimizer')
import mhe_optimizer
solver = mhe_optimizer.solver()
result = solver.run(p=P)
print(result)
Q_star = result.solution
print(result.solve_time_ms)

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
    phi += u2 * ts
    theta += (u1 / L) * math.tan(phi) * ts
    
    # Calculate the new position of the car
    x += u1 * math.cos(theta) * ts
    y += u1 * math.sin(theta) * ts
    
    
    return x, y, theta, phi

polozenia = []
polozenia = Q_star
x = 0.0
y = 0.0
phi = 0.0
th = 0.0

T = 400
# print(f'predfkosci len  {len(Q_star)}')
# Q_star = Q_star[:800]

u1_list = U[0::2]
u2_list = U[1::2]


# for i in range(T):
#     # print(i)
#     x, y, th, phi = update_position(x, y, th, phi, u1_list[i], u2_list[i], 2.5, 0.1)
    
#     polozenia.append(phi)
#     polozenia.append(th)
#     polozenia.append(x)
#     polozenia.append(y)

import matplotlib.pyplot as plt


# print(f'predfkosci {Q_star}')
x = polozenia[2::4]
y = polozenia[3::4]
th = polozenia[1::4]
x1 = Q[2::4]
y1 = Q[3::4]
th1 = Q[1::4]

# plt.figure()
# plt.plot(th, label='th', marker='o')
# plt.plot(th1, label='th1', marker='o')
# plt.title('th')
# plt.legend()

# plt.figure()
# plt.plot(y)
# plt.plot(y1)
# plt.title('x')

plt.figure()
plt.plot(x, y, marker='o')
plt.plot(x1, y1, marker='x')
plt.title('xy')

# plt.figure()
# plt.plot(U[0::2], marker='o')
# plt.plot(Q_star[0::2], marker='o')
# plt.title('u1')

# plt.figure()
# plt.plot(U[1::2], marker='o')
# plt.plot(Q_star[1::2], marker='o')
# plt.title('u2')

plt.show()
