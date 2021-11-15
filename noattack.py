from control.matlab import ss, lsim, linspace
import matplotlib.pyplot as plt
from lib.PID import PID
import numpy as np
from random import random, randint
import pandas as pd

# # system dynamics
# J = 0.01
# b = 0.1
# Ke = 0.01
# Kt = 0.01
# R = 1
# L = 0.5
#
# A = [[-b / J, Kt / J], [-Ke / L, -R / L]]
# B = [[0], [1 / L]]
# C = [1, 0]
# sys = ss(A, B, C, 0)
#
#
# # timing parameters
# t_total = 100
# dt = 0.05
# slot = int(t_total / dt)
# t_arr = linspace(0, t_total, slot+1)
#
#
# # reference
# ref = [5] * 500 + [4] * 500 + [5] * 500 + [6] * 501
#
#
# # yout, T, xout = lsim(sys, 1, t_arr)
# # print(yout.shape, T.shape, xout.shape)
# # plt.plot(T, yout)
# # plt.show()
#
# # PID
# pid = PID(P=19, I=35, D=0.5, current_time=0)
# pid.setWindup(100)
# pid.setSampleTime(dt)
#
#
# x_0 = [[0], [0]]
# y_real_arr = []
# y_real = 0
# cin_arr = []
#
# for i in range(0, slot+1):
#     y_real_arr.append(y_real)
#     pid.SetPoint = ref[i]
#     pid.update(feedback_value=y_real, current_time=i * dt)
#     cin = pid.output
#     cin_arr.append(cin)
#     yout, T, xout = lsim(sys, cin, [0, dt], x_0)
#     y_real = yout[-1]
#     x_0 = xout[-1, :].T
#
# plt.plot(t_arr, y_real_arr, t_arr, ref, t_arr, cin_arr)
# plt.show()


# # ------- RLC Circuit -------------
#
# # model
# R = 10000
# L = 0.5
# C = 0.0001
#
# A = [[0, 1 / C], [-1 / L, -R / L]]
# B = [[0], [1 / L]]
# C = [[1, 0]]
# D = [[0]]
#
# sys = ss(A, B, C, 0)
#
# # timing parameters
# t_total = 100
# dt = 0.05
# slot = int(t_total / dt)
# t_arr = linspace(0, t_total, slot+1)
#
# # reference
# ref = [5] * 500 + [4] * 500 + [5] * 500 + [6] * 501
#
# # PID
# pid = PID(P=5, I=5, D=0, current_time=0)
# pid.setWindup(100)
# pid.setSampleTime(dt)
#
#
# x_0 = [[0], [0]]
# y_real_arr = []
# y_real = 0
# cin_arr = []
#
# for i in range(0, slot+1):
#     y_real_arr.append(y_real)
#     pid.SetPoint = ref[i]
#     pid.update(feedback_value=y_real, current_time=i * dt)
#     cin = pid.output
#     cin_arr.append(cin)
#     yout, T, xout = lsim(sys, cin, [0, dt], x_0)
#     y_real = yout[-1]
#     x_0 = xout[-1, :].T
#
# # plt.plot(t_arr, y_real_arr, t_arr, ref, t_arr, cin_arr)
# plt.plot(t_arr, y_real_arr, t_arr, ref)
# plt.show()


# # ------- DC Motor Position -------------
#
# # model
# J = 0.01
# b = 0.1
# K = 0.01
# R = 1
# L = 0.5
#
# A = [[0, 1, 0],
#      [0, -b / J, K / J],
#      [0, -K / L, -R / L]]
# B = [[0], [0], [1 / L]]
# C = [[1, 0, 0]]
# D = [[0]]
#
# sys = ss(A, B, C, 0)
#
# # timing parameters
# t_total = 100
# dt = 0.05
# slot = int(t_total / dt)
# t_arr = linspace(0, t_total, slot+1)
#
# # reference
# ref = [5] * 500 + [4] * 500 + [5] * 500 + [6] * 501
#
# # PID
# pid = PID(P=11, I=0, D=5, current_time=0)
# pid.setWindup(100)
# pid.setSampleTime(dt)
#
#
# x_0 = [[0], [0], [0]]
#
# x_arr = []
# y_real_arr = []
# y_real = 0
# cin_arr = []
#
# for i in range(0, slot+1):
#     y_real_arr.append(y_real)
#     pid.SetPoint = ref[i]
#     pid.update(feedback_value=y_real, current_time=i * dt)
#     cin = pid.output
#     cin_arr.append(cin)
#     yout, T, xout = lsim(sys, cin, [0, dt], x_0)
#     y_real = yout[-1]
#     x_0 = xout[-1, :].T
#     x_arr.append(x_0)
#
# # plt.plot(t_arr, y_real_arr, t_arr, ref, t_arr, cin_arr)
# plt.plot(t_arr, x_arr)
# # plt.plot(t_arr, y_real_arr, t_arr, ref)
# plt.show()


# ------- Aircraft Pitch -------------

# model
A = [[-0.313, 56.7, 0],
     [-0.0139, -0.426, 0],
     [0, 56.7, 0]]
B = [[0.232], [0.0203], [0]]
C = [[0, 0, 1]]
D = [[0]]
sys = ss(A, B, C, 0)

# timing parameters
t_total = 1000
dt = 0.05
slot = int(t_total / dt)
t_arr = linspace(0, t_total, slot+1)

# reference
ref = []
while len(ref) <= t_total / dt:
    a_ref = random() * 10
    duration = randint(10, 100)
    for _ in range(duration):
        ref.append(a_ref)

# PID
pid = PID(P=14, I=0.8, D=5.7, current_time=0)
pid.setWindup(100)
pid.setSampleTime(dt)

x_0 = [[0], [0], [0]]

x_arr = []
y_real_arr = []
y_real = 0
cin_arr = []

for i in range(0, slot+1):
    y_real_arr.append(y_real)
    pid.SetPoint = ref[i]
    pid.update(feedback_value=y_real, current_time=i * dt)
    cin = pid.output
    cin_arr.append(cin)
    yout, T, xout = lsim(sys, cin, [0, dt], x_0)
    y_real = yout[-1]
    x_0 = xout[-1, :].T
    x_arr.append(x_0)

x_arr = np.array(x_arr)
y_real_arr = np.array(y_real_arr).reshape([-1,1])
cin_arr = np.array(cin_arr).reshape([-1,1])
ref = np.array(ref).reshape([-1,1])[:cin_arr.shape[0]]

data = np.hstack((ref,cin_arr,y_real_arr,x_arr))
# print(f'ref:{ref.shape},x_arr:{x_arr.shape},y_real_arr:{y_real_arr.shape},cin_arr:{cin_arr.shape}')
print(f'data:{data.shape}')
df = pd.DataFrame(data=data, columns=['Reference', 'Control', 'Measure', 'Sate_1', 'Sate_2', 'Sate_3'])
df.to_csv('aircraft_pitch.csv', index=False)
exit(0)
# plt.plot(t_arr, y_real_arr, t_arr, ref, t_arr, cin_arr)
plt.plot(t_arr, x_arr)
# plt.plot(t_arr, y_real_arr, t_arr, ref)
plt.show()


# pd = pandas()
# df.to_csv(os.path.join(ROOT_DIR, "results", 'testbed', scenario, f'loss-{model_name}.csv'), index=False)

# # ------- Quadrotor -------------
#
# # model
# g = 9.81
# m = 0.468
# A = [[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
#      [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
#      [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [0, -g, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [g, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#      [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
#      [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]]
# B = [[0], [0], [0], [0], [0], [0], [0], [0], [1 / m], [0], [0], [0]]
# C = [[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]]
# D = [[0], [0], [0], [0], [0], [0]]
# sys = ss(A, B, C, 0)
#
# # timing parameters
# t_total = 100
# dt = 0.05
# slot = int(t_total / dt)
# t_arr = linspace(0, t_total, slot+1)
#
# # reference
# ref = [5] * 500 + [4] * 500 + [5] * 500 + [6] * 501
#
# # PID
# pid = PID(P=0.1, I=0, D=0.6, current_time=0)
# pid.setWindup(100)
# pid.setSampleTime(dt)
#
#
# x_0 = [[0], [0], [0], [0], [0], [0], [0], [0], [0]]
#
# x_arr = []
# y_real_arr = []
# y_real = [0, 0, 0, 0, 0, 0]
#
# cin_arr = []
#
# for i in range(0, slot+1):
#     y_real_arr.append(y_real)
#     pid.SetPoint = ref[i]
#     pid.update(feedback_value=y_real[-1], current_time=i * dt)
#     cin = pid.output
#     cin_arr.append(cin)
#     yout, T, xout = lsim(sys, cin, [0, dt], x_0)
#     y_real = yout[-1]
#     x_0 = xout[-1, :].T
#     x_arr.append(x_0)
#
# x_arr = np.array(x_arr)
# print(x_arr[100])
# for i in range(x_arr.shape[1]):
#     plt.plot(t_arr, x_arr[:, i], label=i)
# plt.legend()
# plt.show()