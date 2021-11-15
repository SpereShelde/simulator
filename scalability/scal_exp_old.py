import numpy as np
from control.matlab import ss, linspace, c2d
from lib.state_estimation import Estimator

r_state = 348
A = np.genfromtxt('/home/lion/workspace/simulator/scalability/model/A.csv', delimiter=',')
B = np.genfromtxt('/home/lion/workspace/simulator/scalability/model/B.csv', delimiter=',').reshape((r_state, 1))
C = np.genfromtxt('/home/lion/workspace/simulator/scalability/model/C.csv', delimiter=',').reshape((1, r_state))
D = np.zeros((1, 1))

class Exp:
    __slots__ = ['name', 'sysc', 'Ts', 'sysd', 'x_0', 'y_0', 'p', 'i', 'd', 'ref',
                 'est', 'slot', 't_arr', 't_attack', 't_detect', 'attacks', 'y_index',
                 'safeset', 'target_set', 'control_limit', 'max_k', 'worst_case_control', 'k', 'epsilon',
                 'sep_graph', 'y_label', 'x_left', 'y_up', 'y_lo']
    count = 0

    def __init__(self, sysc, Ts, epsilon=1e-7):
        self.sysd = c2d(sysc, Ts)
        self.Ts = Ts
        self.sysc = sysc
        self.epsilon = epsilon
        self.est = Estimator(self.sysd, 150, self.epsilon)
        self.y_index = None
        self.worst_case_control = 'current'
        self.k = None
        self.y_up = None
        self.y_lo = None


# ------- Vehicle Turing -------------
state = 5
sysc = ss(A[0:state, 0:state], B[0:state, 0:1], C[0:1, 0:state], D)
dt = 10
exp = Exp(sysc, dt)
exp.name = str(state)
exp.x_0 = np.zeros((state,1))
exp.y_0 = 0

# control
exp.p = 7.721e-05
exp.i = 7.721e-06
exp.d = 0.000193

# time
t_total = 6000
exp.slot = int(t_total / dt)
exp.t_arr = linspace(0, t_total, exp.slot + 1)
exp.t_attack = int(3000 / dt)
exp.t_detect = int(3500 / dt)

# reference
exp.ref = [0.5] * 601

# recovery
exp.safeset = {'lo': [-2.7], 'up': [2.7]}
exp.target_set = {'lo': [0.9], 'up': [1.1]}
exp.control_limit = {'lo': [-5], 'up': [5]}
exp.max_k = 20
exp.worst_case_control = 'control_up'

# graph
exp.sep_graph = True
exp.y_label = 'Speed Difference'
exp.x_left = 3.9
exp.y_up = 3
exp.y_lo = -0.5

# yout, T = step(sys)
# plt.plot(T, yout)
# plt.show()

print('nn')