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

P = U + U_odom + U_imu

import sys
sys.path.insert(1, '/home/maja/Documents/mgr/mhe_optimizers/mhe_optimizer')
import mhe_optimizer
solver = mhe_optimizer.solver()
result = solver.run(p=P)
Q_star = result.solution
print(result.solve_time_ms)

import matplotlib.pyplot as plt
plt.plot(U[0::2])
plt.plot(Q_star[0::2])
plt.title('u1')

plt.figure()
plt.plot(U[1::2])
plt.plot(Q_star[1::2])
plt.title('u2')

plt.show()
