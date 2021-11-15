from control.matlab import lsim, linspace
import numpy as np
import matplotlib.pyplot as plt
from lib.PID import PID
from scal_exp import sys


# timing parameters
t_total = 6000
dt = 10
slot = int(t_total / dt)
t_arr = linspace(0, t_total, slot+1)


# reference
ref = [0.5] * 601


# yout, T, xout = lsim(sys, 1, t_arr)
# print(yout.shape, T.shape, xout.shape)
# plt.plot(T, yout)
# plt.show()

# PID
pid = PID(P=7.721e-05, I=7.721e-06, D=0.000193, current_time=0)
pid.setWindup(10000)
pid.setSampleTime(dt)


x_0 = np.zeros((100, 1))
y_real_arr = []
y_real = 0
cin_arr = []

for i in range(0, slot+1):
    y_real_arr.append(y_real)
    # sensor attack here

    pid.SetPoint = ref[i]
    pid.update(feedback_value=y_real, current_time=i * dt)
    cin = pid.output
    cin_arr.append(cin)
    yout, T, xout = lsim(sys, cin, [0, dt], x_0)
    y_real = yout[-1]
    x_0 = xout[-1, :].T

plt.plot(t_arr, y_real_arr, t_arr, ref)
plt.show()


