from control.matlab import ss, linspace, c2d
from functools import partial
from lib.state_estimation import Estimator
import numpy as np
import math


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

# model
A = [[-25 / 3]]
B = [[5]]
C = [[1]]
D = [[0]]
sysc = ss(A, B, C, D)
dt = 0.05
vehicle_turning = Exp(sysc, dt)
exp = vehicle_turning
exp.name = 'Vehicle Turning'
exp.x_0 = [[1]]
exp.y_0 = 1

# control
exp.p = 0.5
exp.i = 7
exp.d = 0

# time
t_total = 10
exp.slot = int(t_total / dt)
exp.t_arr = linspace(0, t_total, exp.slot + 1)
exp.t_attack = int(4 / dt)
exp.t_detect = int(5.6 / dt)

# reference
exp.ref = [0] * 251 + [1] * 250

# attacks
exp.attacks = {'modification': {}, 'delay': {}, 'replay': {}}
exp.attacks['modification']['func'] = partial((lambda x, y: x - y), y=1.5)
exp.attacks['delay']['step'] = 50
exp.attacks['replay']['start'] = 0
exp.attacks['replay']['end'] = int(6 / dt)

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

# ------- RLC Circuit -------------

# model
R = 10000
L = 0.5
C = 0.0001

A = [[0, 1 / C], [-1 / L, -R / L]]
B = [[0], [1 / L]]
C = [[1, 0]]
D = [[0]]
sysc = ss(A, B, C, D)
dt = 0.02
RLC_circuit = Exp(sysc, dt)
exp = RLC_circuit
exp.name = 'RLC Circuit'
exp.x_0 = [[0], [0]]
exp.y_0 = 0

# control
exp.p = 5
exp.i = 5
exp.d = 0

# time
t_total = 10
exp.slot = int(t_total / dt)
exp.t_arr = linspace(0, t_total, exp.slot + 1)
exp.t_attack = int(3 / dt)
exp.t_detect = int(4.3 / dt)

# reference
exp.ref = [2] * 201 + [3] * 200 + [2] * 100

# attacks
exp.attacks = {'modification': {}, 'delay': {}, 'replay': {}}
exp.attacks['modification']['func'] = partial((lambda x, y: x - y), y=2.5)
exp.attacks['delay']['step'] = 50
exp.attacks['replay']['start'] = 0
exp.attacks['replay']['end'] = int(5 / dt)

# recovery
exp.safeset = {'lo': [0, -5], 'up': [7, 5]}
exp.target_set = {'lo': [2.9, -5], 'up': [3.1, 5]}
exp.control_limit = {'lo': [-15], 'up': [15]}
exp.max_k = 20
exp.worst_case_control = 'control_up'

# graph
exp.sep_graph = True
exp.y_label = 'Capacitor Voltage'
exp.x_left = 3.5
exp.y_up = 6.5
exp.y_lo = 0

# ------- DC Motor Position -------------

# model
J = 0.01
b = 0.1
K = 0.01
R = 1
L = 0.5

A = [[0, 1, 0],
     [0, -b / J, K / J],
     [0, -K / L, -R / L]]
B = [[0], [0], [1 / L]]
C = [[1, 0, 0]]
D = [[0]]
sysc = ss(A, B, C, D)
dt = 0.1
dc_motor_position = Exp(sysc, dt)
exp = dc_motor_position
exp.name = 'DC Motor Position'
exp.x_0 = [[0], [0], [0]]
exp.y_0 = 0

# control
exp.p = 11
exp.i = 0
exp.d = 5

# time
t_total = 12
exp.slot = int(t_total / dt)
exp.t_arr = linspace(0, t_total, exp.slot + 1)
exp.t_attack = int(6 / dt)
exp.t_detect = int(10 / dt)

# reference
exp.ref = [math.pi/2] * 71 + [-math.pi/2] * 50

# attacks
exp.attacks = {'modification': {}, 'delay': {}, 'replay': {}}
exp.attacks['modification']['func'] = partial((lambda x, y: x + y), y=2)
exp.attacks['delay']['step'] = 10
exp.attacks['replay']['start'] = 0
exp.attacks['replay']['end'] = int(6 / dt)

# recovery
exp.safeset = {'lo': [-4, -100, -100], 'up': [4, 100, 100]}
exp.target_set = {'lo': [-math.pi/2-0.1, -100, -100], 'up': [-math.pi/2+0.1, 100, 100]}
exp.control_limit = {'lo': [-20], 'up': [20]}
exp.max_k = 20
exp.worst_case_control = 'current'

# graph
exp.sep_graph = True
exp.y_label = 'Rotation Angle'
exp.x_left = 6
exp.y_up = 3
exp.y_lo = -4

# ------- Aircraft Pitch -------------

# model
A = [[-0.313, 56.7, 0],
     [-0.0139, -0.426, 0],
     [0, 56.7, 0]]
B = [[0.232], [0.0203], [0]]
C = [[0, 0, 1]]
D = [[0]]
sysc = ss(A, B, C, D)
dt = 0.02
aircraft_pitch = Exp(sysc, dt)
exp = aircraft_pitch
exp.name = 'Aircraft Pitch'
exp.x_0 = [[0], [0], [0]]
exp.y_0 = 0

# control
exp.p = 14
exp.i = 0.8
exp.d = 5.7

# time
t_total = 10
exp.slot = int(t_total / dt)
exp.t_arr = linspace(0, t_total, exp.slot + 1)
exp.t_attack = int(3 / dt)
exp.t_detect = int(4.46 / dt)

# reference
exp.ref = [0.5] * 201 + [0.7] * 200 + [0.5] * 100
# exp.ref = [0.5] * 161 + [1] * 30 + [0.5] * 10 + [1] * 200 + [0.5] * 100

# attacks
exp.attacks = {'modification': {}, 'delay': {}, 'replay': {}}
exp.attacks['modification']['func'] = partial((lambda x, y: x + y), y=0.68)
exp.attacks['delay']['step'] = 50
exp.attacks['replay']['start'] = int(1 / dt)
exp.attacks['replay']['end'] = int(2 / dt)

# recovery
exp.safeset = {'lo': [-100, -100, 0], 'up': [100, 100, 2]}
exp.target_set = {'lo': [-100, -100, 0.68], 'up': [100, 100, 0.72]}
exp.control_limit = {'lo': [-20], 'up': [20]}
exp.max_k = 40
exp.worst_case_control = 'current'

# graph
exp.sep_graph = True
exp.y_label = 'Pitch'
exp.x_left = 3.8
exp.y_up = 2
exp.y_lo = -0.3

# ------- Quadrotor -------------

# model
g = 9.81
m = 0.468
A = [[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, -g, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [g, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]]
B = [[0], [0], [0], [0], [0], [0], [0], [0], [1 / m], [0], [0], [0]]
C = [[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]]
D = [[0], [0], [0], [0], [0], [0]]
sysc = ss(A, B, C, D)
dt = 0.02
quadrotor = Exp(sysc, dt)
exp = quadrotor
exp.name = 'Quadrotor'
exp.x_0 = [[0], [0], [0], [0], [0], [0], [0], [0], [0]]
exp.y_0 = np.array([0, 0, 0, 0, 0, 0])
exp.y_index = 5

# control
exp.p = 0.1
exp.i = 0
exp.d = 0.6

# time
t_total = 30
exp.slot = int(t_total / dt)
exp.t_arr = linspace(0, t_total, exp.slot + 1)
exp.t_attack = int(12 / dt)
exp.t_detect = int(13.1 / dt)

# reference
exp.ref = [2] * 601 + [4] * 600 + [2] * 300

# attacks
exp.attacks = {'modification': {}, 'delay': {}, 'replay': {}}
exp.attacks['modification']['func'] = partial((lambda x, y: x - y), y=2)
exp.attacks['delay']['step'] = 50
exp.attacks['replay']['start'] = 0
exp.attacks['replay']['end'] = int(5 / dt)
exp.worst_case_control = 'current'

# recovery
exp.safeset = {'lo': [-100] * 8 + [-1], 'up': [100] * 8 + [8]}
exp.target_set = {'lo': [-100.0] * 8 + [3.9], 'up': [100.0] * 8 + [4.1]}
exp.control_limit = {'lo': [-50], 'up': [50]}
exp.max_k = 20

# graph
exp.sep_graph = True
exp.y_label = 'Altitude'
exp.x_left = 12
exp.y_up = 8.2
exp.y_lo = 1.8