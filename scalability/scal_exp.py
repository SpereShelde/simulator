import numpy as np
from control.matlab import ss, linspace, c2d
from lib.state_estimation import Estimator


class Exp:
    __slots__ = ['name', 'num', 'sysc', 'Ts', 'sysd', 'x_0', 'y_0', 'p', 'i', 'd', 'ref', 'attack_value',
                 'est', 'slot', 't_arr', 't_attack', 't_detect', 'attacks', 'y_index', 'y_point',
                 'safeset', 'target_set', 'control_limit', 'max_k', 'worst_case_control', 'k', 'epsilon',
                 'sep_graph', 'y_label', 'x_left', 'y_up', 'y_lo']
    count = 0

    def __init__(self, sysc, Ts, epsilon=1e-7):
        self.sysd = c2d(sysc, Ts)
        self.Ts = Ts
        self.sysc = sysc
        self.epsilon = epsilon
        self.est = Estimator(self.sysd, 300, self.epsilon)
        self.y_index = None
        self.worst_case_control = 'current'
        self.k = None
        self.y_up = None
        self.y_lo = None


state_num = 45
A = np.zeros((state_num, state_num))
tmp = np.array([[1], [-2], [1]])
A[0:2, 0:1] = tmp[1:3, 0:1]
for i in range(state_num-2):
    A[i:i+3, i+1:i+2] = tmp
A[state_num-2:state_num, state_num-1:state_num] = tmp[0:2, 0:1]

if state_num==2:
    A = np.array([[-2,1], [1,-2]])


B = np.zeros((state_num, 1))
u_point = (state_num+1)//3-1
B[u_point, 0] = 1

C = np.zeros((1, state_num))
y_point = (state_num+1)//3*2-1
C[0, y_point] = 1

D = np.zeros((1, 1))

dt = 0.2
sys = ss(A, B, C, D)
exp = Exp(sys, dt)
exp.num = state_num
exp.name = str(state_num)
exp.x_0 = np.zeros((state_num, 1))
exp.y_0 = 0
exp.y_point = y_point

# control
p_offset = {40: -0.3, 30: -0.09, 25: -0.04}
exp.p = 0.00000099 * state_num * state_num * state_num * state_num
if state_num in p_offset:
    exp.p += p_offset[state_num]
print('p=', exp.p)
exp.i = 0
exp.d = 0

exp.attack_value = 50

# time
t_total = 80
exp.slot = int(t_total / dt)
exp.t_arr = linspace(0, t_total, exp.slot + 1)
exp.t_attack = int(0 / dt)
exp.t_detect = int(20 / dt)

# reference
ref_value = 15
exp.ref = [ref_value] * int(t_total/dt+1)

# recovery
exp.safeset = {'lo': [-1000000] * state_num, 'up': [1000000] * state_num}
exp.safeset['lo'][y_point] = -1000000
exp.safeset['up'][y_point] = 40
exp.target_set = {'lo': [-1000000] * state_num, 'up': [1000000] * state_num}
error_bound = 8
exp.target_set['lo'][y_point] = ref_value - error_bound
exp.target_set['up'][y_point] = ref_value + error_bound
exp.control_limit = {'lo': [-10000], 'up': [10000]}
exp.max_k = 10
exp.worst_case_control = 'current'

# graph
exp.sep_graph = True
exp.y_label = 'Temp'
exp.x_left = 0
exp.y_up = 50
exp.y_lo = -0.5

# yout, T = step(sys)
# plt.plot(T, yout)
# plt.show()

print('nn')
