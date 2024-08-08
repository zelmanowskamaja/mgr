import math
import numpy as np 

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

def generate_data(T, ts, L):
    P = []
    Q = []
    x = 0.0
    y = 0.0
    th = 0.0
    phi = 0.0
    u1 = 0.0
    u2 = 0.0

    U = []
    U_odom = []
    U_imu = []
    Theta_imu = []
    Q_noise = []

    
    for i in range(0, T):
        
        u1 = 0.2 + 0.2*np.sin(i*0.1)
        u2 = np.cos(0.01*i*4)*0.6
        U.append(u1)
        U.append(u2)
        U_odom.append(u1 + np.random.normal(0, 0.05,  1)[0])
        U_odom.append(u2 + np.random.normal(0, 0.05,  1)[0])
        # U_imu.append(u1 + np.random.normal(0, 0.05,  1)[0])
        U_imu.append(u2 + np.random.normal(0, 0.05,  1)[0])

        x, y, th, phi = update_position(x, y, th, phi, u1, u2, L, ts)

        Q.append(th)
        Q.append(phi)
        Q.append(x)
        Q.append(y)

        Q_noise.append(th + np.random.normal(0, 0.1,  1)[0])
        Q_noise.append(phi + np.random.normal(0, 0.1,  1)[0])
        Q_noise.append(x + np.random.normal(0, 0.1,  1)[0])
        Q_noise.append(y + np.random.normal(0, 0.1,  1)[0])

        
    P = U + U_odom + U_imu 
    return U, U_odom, U_imu, Q_noise, Q



x, y, theta = 0.0, 0.0, 0.0  # Initial position and orientation
s = 0.5 # speed m/s
phi = math.radians(30)  # steering angle in radians
L = 2.5  # Distance between axles
ts = 0.1  # Time step in seconds
T = 400
# Update the car's position

# print(f"Updated position: x={x}, y={y}, theta={math.degrees(theta)} degrees")
U, U_odom, U_imu, Q_noise, Q = generate_data(T, ts, L)

def compute_angular_velocities(positions, linear_velocities, ts):
    angular_velocities = []
    
    for t in range(T - 1):  # Iteracja przez horyzont czasowy T
        theta_t = positions[t * 4]  # theta at time t
        theta_t1 = positions[(t + 1) * 4]  # theta at time t+1

        # Oblicz zmianę kąta
        delta_theta = theta_t1 - theta_t

        # Oblicz prędkość kątową
        omega_t = delta_theta / ts

        angular_velocities.append(omega_t)

    return angular_velocities


angular_velocities = compute_angular_velocities(Q, U, ts)
angular_velocities.insert(0,0.0)
print(len(angular_velocities))


for i in range(T):
    U_imu[i] = angular_velocities[i]
    U_odom[i * 2 + 1] = angular_velocities[i]


import matplotlib.pyplot as plt
x = Q[2::4]
y = Q[3::4]

# Rysowanie wykresu
plt.plot(x, y, marker='o')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Wykres pozycji x i y')
plt.grid(True)
plt.show()

# plt.plot(Q[0::3])
# # plt.plot(Q_star[0::3])
# # plt.plot(Q_noise[0::3])
# plt.ylabel('theta')

# Podział listy U na u1 i u2
u1 = U[::2]
u2 = U[1::2]

# Rysowanie wykresów
plt.figure(figsize=(8, 6))

plt.subplot(2, 1, 1)
plt.plot(u1, marker='o')
plt.title('Wykres u1')
plt.xlabel('Indeks')
plt.ylabel('Wartość')

plt.subplot(2, 1, 2)
plt.plot(u2, marker='o')
plt.title('Wykres u2')
plt.xlabel('Indeks')
plt.ylabel('Wartość')

plt.tight_layout()
plt.show()

# plt.figure()
# plt.plot(Q[1::3], Q[2::3])
# # plt.plot(Q_star[1::3], Q_star[2::3])
# # plt.plot(Q_noise[1::3], Q_noise[2::3])
# plt.gca().set_aspect('equal')
# plt.ylabel('xy')

# plt.show()
x_positions = [0.0]
y_positions = [0.0]
theta_positions = [0.0]

for t in range(T - 1):
    x_prev = x_positions[-1]
    y_prev = y_positions[-1]
    theta_prev = theta_positions[-1]

    # Prędkości
    v = U[2*t]
    omega = angular_velocities[t]

    # Nowe pozycje
    theta_new = theta_prev + omega * ts
    x_new = x_prev + v * np.cos(theta_new) * ts
    y_new = y_prev + v * np.sin(theta_new) * ts
    
    x_positions.append(x_new)
    y_positions.append(y_new)
    theta_positions.append(theta_new)

plt.figure(figsize=(10, 5))
plt.plot(x_positions, y_positions, label='Trajectory')
plt.scatter([x], [y], color='red', label='Reference Point')
plt.title('Trajectory of the Vehicle')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.legend()
plt.grid()
plt.show()

import pandas as pd

# filename = "dane.csv"

# data = {
#     "U": U,
#     "U_odom": U_odom,
#     "U_imu": U_imu,
#     "Q": Q
# }

# # Zapis danych do pliku CSV
# df = pd.DataFrame(data)
# df.to_csv("dane.csv", index=False)

with open('listy.txt', 'w') as file:
    for lista in [U, U_odom, U_imu, Q_noise, Q]:
        file.write(f"{len(lista)}," + ",".join(map(str, lista)) + "\n")


