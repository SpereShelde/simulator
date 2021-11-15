import time
import glpk
import numpy as np
from control.matlab import c2d, ss


def recovery(sysd, k, initial_set_lo, initial_set_up, target_set_lo, target_set_up, safe_set_lo, safe_set_up,
             control_lo, control_up, debug=True):

    m = sysd.A.shape[0]
    n = sysd.B.shape[1]

    start = time.time()
    # ---------------prepare matrix
    row_num = m * k
    col_num = (k + 1) * m + n * k
    matrix = np.zeros((row_num, col_num), np.float)
    # fill Ad
    for i in range(0, m * k, m):
        matrix[i:i + m, i:i + m] = -sysd.A
    # fill 1
    for i in range(0, m * k):
        matrix[i, i + m] = 1
    # fill Bd
    for i in range(0, k):
        x = i * m
        y = m * (k + 1) + n * i
        matrix[x:x + m, y:y + n] = -sysd.B

    # ---------------- def problem
    lp = glpk.LPX()
    lp.name = 'recovery'
    lp.obj.maximize = False

    # ---------------- rows
    lp.rows.add(row_num)
    for r in lp.rows:
        r.bounds = 0

    # ---------------- cols
    lp.cols.add(col_num)
    lp.obj[:] = [1] * col_num
    # initial set bound
    for i in range(m):
        lp.cols[i].bounds = initial_set_lo[i, 0], initial_set_up[i, 0]
    # safe set bound
    for i in range(1, k):
        for j in range(m):
            lp.cols[i * m + j].bounds = safe_set_lo[j], safe_set_up[j]
    # target set bound
    for i in range(m):
        lp.cols[m * k + i].bounds = target_set_lo[i], target_set_up[i]
    # control bound
    for i in range(k):
        for j in range(n):
            lp.cols[m * (k + 1) + i * n + j].bounds = control_lo[j], control_up[j]

    # -----------------  load matrix
    lp.matrix = matrix.flatten().tolist()

    # -----------------  solve problem
    solve_start = time.time()
    retval = lp.simplex()
    assert retval is None
    if lp.status != 'opt':
        print('No solution found!')
        raise RuntimeError('Error while solving...: ' + lp.status)

    end = time.time()
    solve = end - solve_start
    total = end - start
    if debug:
        print('Solve=', solve, 'seconds; ', 'Total=', total, 'seconds.')

        # print state for debug
        state_lst = [r.primal for r in lp.cols[:m * (k + 1)]]
        state = np.array(state_lst).reshape((k + 1, -1))
        print(state.T)

    # return n x k control input
    control_lst = [r.primal for r in lp.cols[m * (k + 1):]]
    control = np.array(control_lst).reshape((k, -1))
    if debug:
        print(control.T)
    return control.T, total-solve, solve


if __name__=="__main__":
    A = [[-10, 1], [-0.02, -2]]
    B = [[0], [2]]
    C = [1, 0]
    sys = ss(A, B, C, 0)
    dt = 0.02
    initial_set_lo = np.array([[7.999902067622], [79.998780693465]])
    initial_set_up = np.array([[7.999902067622887], [79.998780693465960]])
    target_set_lo = [3.9, -100]
    target_set_up = [4.1, 100]
    safe_set_lo = [1, -150]
    safe_set_up = [8, 150]
    control_lo = [-150]
    control_up = [150]
    sysd = c2d(sys, dt)
    recovery(sysd, 10, initial_set_lo, initial_set_up, target_set_lo, target_set_up, safe_set_lo, safe_set_up,
             control_lo, control_up)

    # Solve= 9.989738464355469e-05 seconds;  Total= 0.00031638145446777344 seconds.
    # [[ 7.99990207  7.9148632   7.68504186  7.40124651  7.0725327   6.65702284
    #    6.17619305  5.64740177  5.08464488  4.49917359  3.9       ]
    #  [79.99878069 70.97727415 62.30956903 60.16498737 51.92145991 44.00131259
    #   36.39189513 29.08104571 22.05707325 15.30874015  8.82524552]]
    # [[-150.         -150.            7.69099862 -150.         -150.
    #   -150.         -150.         -150.         -150.         -150.        ]]
