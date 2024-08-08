import sys
import opengen as og
import casadi.casadi as cs
import numpy as np
sys.path.insert(1, '/mhe_optimizers')

ts = 0.1 
T = 10  # Zmniejszona liczba kroków czasowych dla testowania
nq = 4
nu = 2

def unicycle_sys_casadi(q, u): # phi, yaw , x , y , u1, u2, 
    dq = cs.vertcat(
        ts*u[1],
        ts*u[0]*1/2.5*cs.tan(q[0]),
        ts*u[0]*cs.cos(q[1]),
        ts*u[0]*cs.sin(q[1]))
    return dq

def dynamics_casadi(q, u):
    dq = unicycle_sys_casadi(q, u)
    return q + dq  # phi, yaw, x, y 

def theta_difference(theta_2, theta_1):
    v_1 = np.array([np.cos(theta_1), np.sin(theta_1)])
    v_2 = np.array([np.cos(theta_2), np.sin(theta_2)])
    angle_diff = np.arctan2(v_1[0]*v_2[1] - v_1[1]*v_2[0], v_1[0]*v_2[0] + v_1[1]*v_2[1])
    return angle_diff

# Symboliczne zmienne
Q_0 = cs.MX.sym('Q0', nq)  # wy  polozenie w chwili zerowej
U_hat = cs.MX.sym('Uhat', nu*T)  # wy horyzont predkosci 
U = cs.MX.sym('U', nu*T)  # we
U_odom = cs.MX.sym('Uodom', nu*T)  # we
U_imu = cs.MX.sym('Uimu', nu*T)  # we

# Macierz podawana do solvera 
P = cs.vertcat(U, U_odom, U_imu) 

max_u1_velocity = 0.5 
max_u2_velocity = 0.6
velocity_margin_factor = 1.5

V = 0.0 # initial_guess 
Q = Q_0 
Q_list = [Q]  # initialize the list with the initial position

# Debugging: Check types of initial symbolic variables
print(f"Q_0 type: {type(Q_0)}")
print(f"U_hat type: {type(U_hat)}")
print(f"U type: {type(U)}")
print(f"U_odom type: {type(U_odom)}")
print(f"U_imu type: {type(U_imu)}")

for i in range(T): 
    Q_pred = dynamics_casadi(Q, U_hat[i*nu:(i+1)*nu])
    Q_list.append(Q_pred)
    
    e_odom = Q_pred - Q_list[i]
    weight_odom = 0.98
    W_odom  = weight_odom*cs.MX.eye(nq)  # adjust to position size
    odom_norm = e_odom.T @ W_odom @ e_odom

    V += odom_norm
    Q = Q_pred  # update Q

Q_hat = cs.vertcat(*Q_list)

# Debugging: Check types of final symbolic variables
print(f"Q_hat type: {type(Q_hat)}")
print(f"P type: {type(P)}")
print(f"V type: {type(V)}")

u_min = [-max_u1_velocity, -max_u2_velocity] * T
u_max = [max_u1_velocity, max_u2_velocity] * T
bounds = og.constraints.Rectangle(u_min, u_max)

penalty_constraints = cs.vertcat([])

O_vars = cs.vertcat(Q_hat, Q_0)

# Debugging: Check types of O_vars
print(f"O_vars type: {type(O_vars)}")

problem = og.builder.Problem(O_vars, P, V) \
    .with_constraints(bounds) \
    .with_penalty_constraints(penalty_constraints)

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

# Debugging: Check types before building
print(f"problem type: {type(problem)}")
print(f"build_config type: {type(build_config)}")
print(f"meta type: {type(meta)}")
print(f"solver_config type: {type(solver_config)}")

builder.build()
print('built')

print(f'P.shape = {P.shape}')
print(f'O_vars.shape = {O_vars.shape}')
