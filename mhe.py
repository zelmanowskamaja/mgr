import sys
import opengen as og
import casadi.casadi as cs
import numpy as np
sys.path.insert(1, '/mhe_optimizers')


ts = 0.1 
T = 400
nq = 4
nu = 2
L = 2.5  # Długość pojazdu


# def dynamics_casadi(q, u):
#     dq = ackerman_sys_casadi(q, u)
#     return q + dq  #phi, yaw, x, y 
def normalize_angle(angle):
    """
    Normalizuje kąt do zakresu [-pi, pi].
    """
    return cs.fmod(angle + cs.pi, 2 * cs.pi) - cs.pi


def dynamics_casadi(q, u):  # phi, yaw, x, y, u1, u2
    # ts - krok czasowy
    # u[0] - prędkość liniowa
    # u[1] - kąt skrętu
    # q[0] - phi (kąt skrętu)
    # q[1] - yaw (kąt obrotu pojazdu)
    # L - długość między osiami pojazdu

    phi = q[0] + ts * u[1]  # Aktualizacja kąta skrętu
    phi = normalize_angle(phi)
    yaw = q[1] + (ts * u[0] / L) * cs.tan(phi)  # Aktualizacja kąta obrotu pojazdu
    yaw = normalize_angle(yaw)
    x = q[2] + ts * u[0] * cs.cos(yaw)  # Aktualizacja pozycji x
    y = q[3] + ts * u[0] * cs.sin(yaw)  # Aktualizacja pozycji y
    
    dq = cs.vertcat(phi, yaw, x, y)
    return dq



def unicycle_sys_casadi(q, u): # yaw , x , y , angvel, lin vel, 
    dq = cs.vertcat(ts*u[0],
        ts*u[1]*cs.cos(q[0]),
        ts*u[1]*cs.sin(q[0]))
    return dq

def dynamics_casadi_unicycle(q, u):  # phi, yaw, x, y, u1, u2
    yaw = q[1] + ts * u[1]   # Aktualizacja kąta obrotu pojazdu
    yaw = normalize_angle(yaw)
    x = q[2] + ts * u[0] * cs.cos(yaw)  # Aktualizacja pozycji x
    y = q[3] + ts * u[0] * cs.sin(yaw)  # Aktualizacja pozycji y
    
    dq = cs.vertcat(yaw, x, y)
    return dq


def dynamics_casadi_unicycle_yaw(q, u):
    return normalize_angle(q[1] + ts*u)  # yaw

"""
returns theta_2 - theta_1
"""
def theta_difference(theta_2, theta_1):
    v_1 = np.array([np.cos(theta_1), np.sin(theta_1)])
    v_2 = np.array([np.cos(theta_2), np.sin(theta_2)])
    angle_diff = np.arctan2(v_1[0]*v_2[1] - v_1[1]*v_2[0], v_1[0]*v_2[0] + v_1[1]*v_2[1])
    return angle_diff

# wy

Q_0 = cs.SX.sym('Q0', nq)  #wy  polozenie w chwili zerowej
Q_hat = cs.SX.sym('Qhat', nq*T)  #wy  polozenie przez horyzont
U_hat = cs.SX.sym('Uhat', nu*T)  #wy horyzont predkosci 


U = cs.SX.sym('U', nu*T)  #we
U_odom = cs.SX.sym('Uodom', nu*T)  #we
U_imu = cs.SX.sym('Uimu', T)  #we
Q_noise = cs.SX.sym('Uimu', nq*T)  #we

# macierz podwana do solvera 
P = cs.vertcat(U, U_odom, U_imu, Q_noise) 

max_u1_velocity = 1.0
max_u2_velocity = 1.0
velocity_margin_factor = 1.5


w_odom_pose = 1.0  #<---weight of pose from imu and odom 
W_odom  = w_odom_pose*cs.SX.eye(2) 
w_imu_theta = 0.8  
V = 0.0 # initial_guess 
penalty_constraints_list = []  # f. kosztu 
# Q = Q_0
# Q_hat[0:4] = Q_0 
Q = cs.vertcat(0.0, 0.0, 0.0, 0.0)  # Inicjalizacja początkowego stanu w Q



# for i in range(T): 
#     e_odom = U_hat[i*nu:(i+1)*nu] - U[i*nu:(i+1)*nu] 
#     weight_odom = 10e6
#     W_odom  = weight_odom*cs.SX.eye(2) 
#     odom_norm = e_odom.T @ W_odom @ e_odom
#     # q_odom = dynamics_casadi(Q_hat[i*nq:(i+1)*nq], U[i*nu:(i+1)*nu])

#     e_imu = U_hat[i*nu:(i+1)*nu] - U_imu[i*nu:(i+1)*nu] 
#     weight_imu = 400
#     W_imu  = weight_imu*cs.SX.eye(2) 
#     imu_norm = e_imu.T @ W_imu @ e_imu
#     # q_imu = dynamics_casadi(Q_hat[i*nq:(i+1)*nq], U_imu[i*nu:(i+1)*nu])

#     e_wheel = U_hat[i*nu:(i+1)*nu] - U_odom[i*nu:(i+1)*nu] 
#     weight_wheel = 400
#     W_wheel  = weight_wheel*cs.SX.eye(2) 
#     wheel_norm = e_wheel.T @ W_wheel @ e_wheel
#     q_wheel = dynamics_casadi(Q_hat[i*nq:(i+1)*nq], U_odom[i*nu:(i+1)*nu])


#     e_position = - Q_noise[i*nq+2:i*nq+4] + Q[2:4]
#     e_position_theta = theta_difference(Q[1], Q_noise[(i)*nq+1])
    
#     position_weight = 100
#     position_theta_norm = e_position_theta*e_position_theta*position_weight 
#     position_norm = (e_position.T@e_position)*position_weight 

#     V += odom_norm + imu_norm + wheel_norm + position_theta_norm + position_norm
#     # Q = dynamics_casadi(Q_hat[i*nq:(i+1)*nq], U_hat[i*nu:(i+1)*nu]) #to przekazac do odpowiedzi
#     Q = dynamics_casadi(Q, U_hat[i*nu:(i+1)*nu]) #to przekazac do odpowiedzi

#     # e_odom_q = q_odom - 
    
    
    
    
#     # Q_hat[(i+1)*nq:(i+2)*nq] = dynamics_casadi(Q_hat[i*nq:(i+1)*nq], U_hat[i*nu:(i+1)*nu])

#     # phi, yaw, x, y
#     # Q_hat[i*nq:(i+1)*nq] = Q 
  
for i in range(T): 
    Q_imu = dynamics_casadi_unicycle_yaw(Q, U_imu[i])
    e_imu = Q_hat[(i*nq)+ 1] - Q_imu
    weight_imu = 4 #400*ts^2
    W_imu  = weight_imu*cs.SX.eye(1)  # adjust to position size
    imu_norm = e_imu.T @ W_imu @ e_imu

    Q_wheel = dynamics_casadi_unicycle(Q, U_odom[i*nu:(i+1)*nu])
    e_wheel = Q_hat[(i*nq)+1:(i+1)*nq] - Q_wheel
    weight_wheel = 4
    W_wheel  = weight_wheel*cs.SX.eye(3)  # adjust to position size
    wheel_norm = e_wheel.T @ W_wheel @ e_wheel
    
    
    Q = dynamics_casadi(Q, U[i*nu:(i+1)*nu])

    e_odom = Q_hat[(i)*nq:(i+1)*nq] - Q
    # e_odom = Q_hat[(i+1)*nq:(i+2)*nq] - Q_hat[i*nq:(i+1)*nq]
    weight_odom = 10e6
    W_odom  = weight_odom*cs.SX.eye(nq)  # adjust to position size
    odom_norm = e_odom.T @ W_odom @ e_odom

    # Q_noise
    e_q_noise = Q_hat[(i)*nq:(i+1)*nq] - Q_noise[(i)*nq:(i+1)*nq]
    q_weight = 100
    W_q  = q_weight*cs.SX.eye(nq)  # adjust to position size
    q_position_norm = e_q_noise.T @ W_q @ e_q_noise

    # q_position_norm = (e_q_noise.T@e_q_noise)*q_weight 


    V += odom_norm + imu_norm + wheel_norm + q_position_norm
    
    
    # Q = dynamics_casadi(Q, U_hat[i*nu:(i+1)*nu])
    # Q_hat[i*nq:(i+1)*nq] = Q  # update position estimates



u_min = [-max_u1_velocity, -max_u2_velocity] * T
u_max = [max_u1_velocity, max_u2_velocity] * T
bounds = og.constraints.Rectangle(u_min, u_max)

penalty_constraints = cs.vcat(penalty_constraints_list)

O_vars = cs.vertcat(Q_hat)


problem = og.builder.Problem(O_vars, P, V) \
    .with_penalty_constraints(penalty_constraints)

    # .with_constraints(bounds) \
build_config = og.config.BuildConfiguration()  \
    .with_build_directory("/home/maja/Documents/mgr/mhe_optimizers") \
    .with_build_mode("build")                  \
    .with_build_python_bindings()  
meta = og.config.OptimizerMeta()                   \
    .with_optimizer_name("mhe_optimizer")
solver_config = og.config.SolverConfiguration()    \
    .with_tolerance(1e-5)                          \
    .with_delta_tolerance(1e-4)                    \
    .with_initial_penalty(1e3)                     \
    .with_penalty_weight_update_factor(5) 
builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config)
builder.build()
print('built')

print(f'P.shape = {P.shape}')
print(f'O_vars.shape = {O_vars.shape}')





