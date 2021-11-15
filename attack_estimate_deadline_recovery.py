from control.matlab import ss, lsim, linspace, c2d
import numpy as np
import matplotlib.pyplot as plt
from lib.PID import PID
from functools import partial
from lib.state_estimation import Estimator
from lib.recovery import recovery

# system dynamics
J = 0.01
b = 0.1
Ke = 0.01
Kt = 0.01
R = 1
L = 0.5

A = [[-b / J, Kt / J], [-Ke / L, -R / L]]
B = [[0], [1 / L]]
C = [1, 0]
sys = ss(A, B, C, 0)
dt = 0.02
sysd = c2d(sys, dt)
est = Estimator(sysd, 50, 1e-7)

# timing parameters
t_total = 10
slot = int(t_total / dt)
t_arr = linspace(0, t_total, slot + 1)

# attack
t_attack = 250
t_detect = 260
attack_types = ['null', 'modification', 'delay', 'replay']
current_attack = 1


# modification
def sub(x, y):
    return x - y


mod = partial(sub, y=4)
# delay
delay_step = 50
# replay
replay_start = 0
replay_end = 250

# reference
ref = [5] * 201 + [4] * 200 + [5] * 100

# yout, T, xout = lsim(sys, 1, t_arr)
# print(yout.shape, T.shape, xout.shape)
# plt.plot(T, yout)
# plt.show()

# PID
pid = PID(P=19, I=35, D=0.5, current_time=0)
pid.setWindup(100)
pid.setSampleTime(dt)

x_0 = [[0], [0]]
y_real_arr = []
y_attack_arr = []
x_arr = []
y_real = 0
y_attack = 0
cin_arr = []
y_last_normal = 0
x_last_normal = x_0
k = 0
r_control = None
cin=0

for i in range(0, slot + 1):

    y_real_arr.append(y_real)
    x_arr.append(x_0)
    # sensor attack here
    if t_attack <= i <= t_detect:
        if t_attack == i:
            y_last_normal = y_real
            x_last_normal = x_0
        if current_attack == 1:  # modification
            y_attack = mod(y_real)
        elif current_attack == 2:  # delay
            if i - t_attack > delay_step:
                y_attack = y_real_arr[-delay_step]
            else:
                y_attack = y_last_normal
        elif current_attack == 3:  # replay
            delta_i = i - t_attack
            j = replay_start + delta_i % (replay_end - replay_start)
            y_attack = y_real_arr[j]
        else:
            y_attack = y_real
    else:
        y_attack = y_real
    y_attack_arr.append(y_attack)

    if i == t_detect:
        # x_0 estimate
        attack_control = np.array([cin_arr[t_attack:t_detect]])
        x_a = np.array([[item] for item in x_last_normal])
        x0_lo, x0_up = est.estimate(x_a, attack_control)
        print('======== x_0 estimate ==========')
        print('attack_control=', attack_control)
        print("x_last_normal=", x_a)
        print("x0_lo=", x0_lo)
        print("x0_up=", x0_up)
        print("x_0", x_0)
        print('=================')

        # deadline estimate
        print('======== deadline estimate ==========')
        safe_set_lo = [1, -150]
        safe_set_up = [7, 150]
        control = np.array([[cin_arr[-1]]])
        k = est.get_deadline(x0_lo, safe_set_lo, safe_set_up, control, max_k=20)
        print('k=', k)
        print('==================')

        # recovery
        print('======== recovery ==========')
        target_set_lo = [3.91, -100]
        target_set_up = [4.1, 100]
        control_lo = [-150]
        control_up = [150]
        r_control = recovery(sysd, k, x0_lo, x0_up, target_set_lo, target_set_up, safe_set_lo, safe_set_up,
                 control_lo, control_up)
        print("r_control=", r_control)
        print('==================')

    if t_detect <= i < t_detect+k:
        j = i-t_detect
        cin = r_control[0, j]
        pid.clear()
    else:
        pid.SetPoint = ref[i]
        pid.update(feedback_value=y_attack, current_time=i * dt)
        cin = pid.output
    cin_arr.append(cin)
    yout, T, xout = lsim(sys, cin, [0, dt], x_0)
    y_real = yout[-1]
    x_0 = xout[-1, :].T

plt.plot(t_arr, y_real_arr, t_arr, ref)
plt.show()
print(y_real_arr[t_detect+k])

