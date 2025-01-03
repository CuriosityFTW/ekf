import numpy as np
import matplotlib.pyplot as plt

##### FORMAT START #####

# initial position estimation uncertainty (m2)
# P_x_00 = 

# initial velocity estimation uncertainty (m2/s2)
# P_v_00 = 

# ITERATION 0

# VELOCITY
# predicted velocity estimation uncertainty (m2/s2)
# P_v_10 = P_v_00

# POSITION
# predicted position estimation uncertainty (m2)
# P_x_10 = P_x_00

# ITERATION 1

# VELOCITY
# kalman gain for velocity
# K_11 = P_v_10 / (P_v_10 + velne_m_nse**2)

# updated velocity estimation uncertainty (m2/s2)
# P_v_11 = P_v_10*(1 - K_11)

# predicted velocity estimation uncertainty (m2/s2)
# P_v_21 = P_v_11 + (acc_p_nse**2)*(del_t**2) + (abias_p_nse*(del_t**2))**2

# POSITION
# kalman gain for position
# K_11 = P_x_10 / (P_x_10 + posne_m_nse**2)

# updated position estimation uncertainty (m2)
# P_x_11 = P_x_10*(1 - K_11)

# predicted position estimation uncertainty (m2)
# P_x_21 = P_x_11 + P_v_11*(del_t**2) + (acc_p_nse**2)*((del_t**4)/4)

##### FORMAT END #####

# time step for ekf calculations (s)
del_t = 0.04

# number of iterations
n = 300 # for 1/2 mins
# n = 6000 # for 10 mins
#n = 18000 # for 30 mins

# accelerometer rms noise (m/s/s)
acc_p_nse = 0.008832
# acc_p_nse = 0.35

## accelerometer bias instability (m/s/s/s)
abias_p_nse = 0.003

# gps position measurement accuracy (m)
posne_m_nse = 10
#posne_m_nse = 0.5

## gps velocity measurement accuracy (m/s)
velne_m_nse = 0.9
#velne_m_nse = 0.3

K_x = [0]*n
K_v = [0]*n

P_x_initial = (posne_m_nse)**2 ##
P_x_update = [0]*n
P_x_update_sigma = [0]*n
P_x_predict = [0]*n
P_x_predict_sigma = [0]*n

P_v_initial = (velne_m_nse)**2 ##
P_v_update = [0]*n
P_v_predict = [0]*n

# ITERATION 0
# VELOCITY
# predict
P_v_predict[0] = P_v_initial

# POSITION
# predict
P_x_predict[0] = P_x_initial

# ITERATION 1,2,3,...,n
for i in range(1,n):
    # VELOCITY
    # kalman gain
    K_v[i] = P_v_predict[i-1] / (P_v_predict[i-1] + velne_m_nse**2)
    # update
    P_v_update[i] = P_v_predict[i-1] * (1 - K_v[i])
    # predict
    P_v_predict[i] = P_v_update[i] + ((acc_p_nse*del_t)**2) + ((abias_p_nse*(del_t**2))**2)
    # P_v_predict[i] = P_v_update[i] + (0.049481293)**2 + ((abias_p_nse*(del_t**2))**2)
    
    # POSITION
    # kalman gain
    K_x[i] = P_x_predict[i-1] / (P_x_predict[i-1] + posne_m_nse**2)
    # update
    P_x_update[i] = P_x_predict[i-1] * (1 - K_x[i])
    P_x_update_sigma[i] = np.sqrt(P_x_update[i])
    # predict
    P_x_predict[i] = P_x_update[i] + P_v_update[i]*(del_t**2) + ((acc_p_nse*(del_t**2)/2)**2)   
    # P_x_predict[i] = P_x_update[i] + P_v_update[i]*(del_t**2) + (550)**2
    P_x_predict_sigma[i] = np.sqrt(P_x_predict[i])

P_x_update_sigma[0] = P_x_update_sigma[1]
P_x_predict_sigma[0] = np.sqrt(P_x_initial)
K_x[0] = K_x[1]

# Plotting commands
t = [0]*n

t[0] = 0.1

for j in range(1,n):
    t[j] = t[j-1] + 0.1
    if (t[j] > 30):
        break

params = {'figure.figsize': (8, 5), 
          'axes.labelsize': 16,
          'axes.titlesize': 16,
          'xtick.labelsize': 10, 
          'ytick.labelsize': 10}
plt.rcParams.update(params)

# plt.xlabel("Time (s)")
# plt.ylabel("P_x_update_sigma (m)")
# plt.title("Position estimation uncertainty vs time (with delay)")

# plt.plot(t, P_x_update_sigma, color='b', label='P_x_update_sigma')
# plt.plot(t, P_x_predict_sigma, color='r', label='P_x_predict_sigma')
# plt.plot(t, K_x, color='g', label='K_x')

# plt.legend(fontsize=12)
# plt.xlim([0, max(t)])
# plt.ylim([0, max(P_x_update_sigma)+0.25])

# plt.show()

P_x_update_sigma_n = P_x_update_sigma[n-1]
print("Converged position estimation uncertainty (68%) =", P_x_update_sigma_n, "m")
print("Converged position estimation uncertainty (99%) =", 3*P_x_update_sigma_n, "m")

# P_x_predict_sigma_n = P_x_predict_sigma[n-1]
# print("Converged position estimation uncertainty (68%) =", P_x_predict_sigma_n, "m")
# print("Converged position estimation uncertainty (99%) =", 3*P_x_predict_sigma_n, "m")