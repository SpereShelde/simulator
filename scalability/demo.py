import os
from control.matlab import lsim
from lib.PID import PID
from scal_exp import exp
import matplotlib.pyplot as plt
import time
import numpy as np
from lib.recovery import recovery
import traceback

if __name__ == '__main__':
    exps = [exp]
    results = {}
    time_statistics = []

    if not os.path.exists('result'):
        os.mkdir('result')

    for exp in exps:

        print('\n==============================', exp.name, "============================")
        rp = os.path.join('result', exp.name)
        if not os.path.exists(rp):
            os.mkdir(rp)

        # load model
        dt = exp.Ts
        sys = exp.sysc
        pid = PID()
        pid.setWindup(100)
        pid.setSampleTime(dt)
        ref = exp.ref
        t_arr = exp.t_arr
        print(sys)

        # load attack setting
        t_attack = exp.t_attack
        t_detect = exp.t_detect

        # print('---------------------- no attack  --------------------')
        # init pid
        pid.clear()
        pid.setKp(exp.p)
        pid.setKi(exp.i)
        pid.setKd(exp.d)
        pid.last_time = 0
        pid.setControlLimit(exp.control_limit['lo'], exp.control_limit['up'])

        y_real_arr = []
        x_arr = []
        cin_arr = []
        y_real = exp.y_0
        x_0 = exp.x_0

        for i in range(0, exp.slot + 1):
            if exp.y_index:
                y_real = y_real[exp.y_index]
            y_real_arr.append(y_real)
            x_arr.append(x_0)

            pid.SetPoint = ref[i]
            pid.update(feedback_value=y_real, current_time=i * dt)
            cin = pid.output
            cin_arr.append(cin)
            # print(sys)
            yout, T, xout = lsim(sys, cin, [0, dt], x_0)
            # print('yout=', yout)
            y_real = yout[-1]
            if len(xout.shape) == 1:
                x_0 = xout[-1]
            else:
                x_0 = xout[-1, :].T

        if exp.sep_graph:
            plt.figure()
            plt.plot(t_arr, y_real_arr)
            plt.plot(t_arr, ref)
            filename = os.path.join(rp, 'no_attack.jpg')
            plt.savefig(filename)

        try:
            y_arr_list = []
            time_info = {'stateNum': exp.num}

            # print('----------------------' attack w/o recovery   --------------------')
            # init pid
            pid.clear()
            pid.setKp(exp.p)
            pid.setKi(exp.i)
            pid.setKd(exp.d)
            pid.last_time = 0
            pid.setControlLimit(exp.control_limit['lo'], exp.control_limit['up'])

            y_real_arr = []
            y_attack_arr = []
            x_arr = []
            cin_arr = []
            y_real = exp.y_0
            y_attack = exp.y_0
            x_0 = exp.x_0
            y_last_normal = 0
            x_last_normal = x_0

            for i in range(0, exp.slot + 1):
                if exp.y_index:
                    y_real = y_real[exp.y_index]
                y_real_arr.append(y_real)
                x_arr.append(x_0)
                # sensor attack here
                if t_attack <= i:
                    if t_attack == i:
                        y_last_normal = y_real
                        x_last_normal = x_0
                    y_attack = y_real - exp.attack_value
                else:
                    y_attack = y_real
                y_attack_arr.append(y_attack)

                pid.SetPoint = ref[i]
                pid.update(feedback_value=y_attack, current_time=i * dt)
                cin = pid.output
                cin_arr.append(cin)
                # print(sys)
                yout, T, xout = lsim(sys, cin, [0, dt], x_0)
                # print('yout=', yout)
                y_real = yout[-1]
                if len(xout.shape) == 1:
                    x_0 = xout[-1]
                else:
                    x_0 = xout[-1, :].T
            y_arr_list.append(y_real_arr.copy())

            if exp.sep_graph:
                plt.figure()
                plt.plot(t_arr, y_real_arr)
                plt.plot(t_arr, ref)
                plt.ylim(0,50)
                filename = os.path.join(rp, 'no_recovery.jpg')
                plt.savefig(filename)

            # print('-------------------- attack w/ non-real time recovery  ---------------------')
            # init pid
            pid.clear()
            pid.setKp(exp.p)
            pid.setKi(exp.i)
            pid.setKd(exp.d)
            pid.last_time = 0
            pid.setControlLimit(exp.control_limit['lo'], exp.control_limit['up'])

            y_real_arr = []
            y_attack_arr = []
            x_arr = []
            cin_arr = []
            y_real = exp.y_0
            y_attack = exp.y_0
            x_0 = exp.x_0
            y_last_normal = 0
            x_last_normal = x_0

            for i in range(0, exp.slot + 1):
                if exp.y_index:
                    y_real = y_real[exp.y_index]
                y_real_arr.append(y_real)
                x_arr.append(x_0)
                # sensor attack here
                if t_attack <= i < t_detect:
                    if t_attack == i:
                        y_last_normal = y_real
                        x_last_normal = x_0
                    y_attack = y_real - exp.attack_value
                else:
                    y_attack = y_real
                y_attack_arr.append(y_attack)

                pid.SetPoint = ref[i]
                pid.update(feedback_value=y_attack, current_time=i * dt)
                cin = pid.output
                cin_arr.append(cin)
                yout, T, xout = lsim(sys, cin, [0, dt], x_0)
                y_real = yout[-1]
                if len(xout.shape) == 1:
                    x_0 = xout[-1]
                else:
                    x_0 = xout[-1, :].T
            y_arr_list.append(y_real_arr.copy())

            if exp.sep_graph:
                plt.figure()
                plt.plot(t_arr, y_real_arr)
                plt.plot(t_arr, ref)
                plt.ylim(0,50)
                plt.grid()
                filename = os.path.join(rp, 'non_rt_recovery.jpg')
                plt.savefig(filename)

            print('-------------------- attack w/ real-time recovery  --------------------')
            recovered_index = 0
            # init pid
            pid.clear()
            pid.setKp(exp.p)
            pid.setKi(exp.i)
            pid.setKd(exp.d)
            pid.last_time = 0
            pid.setControlLimit(exp.control_limit['lo'], exp.control_limit['up'])

            y_real_arr = []
            y_attack_arr = []
            x_arr = []
            cin_arr = []
            y_real = exp.y_0
            y_attack = exp.y_0
            x_0 = exp.x_0
            y_last_normal = 0
            x_last_normal = x_0
            k = 0
            est = exp.est
            r_control = None
            cin = 0

            for i in range(0, exp.slot + 1):
                if exp.y_index:
                    y_real = y_real[exp.y_index]
                y_real_arr.append(y_real)
                x_arr.append(x_0)
                # sensor attack here
                if t_attack <= i <= t_detect:
                    if t_attack == i:
                        y_last_normal = y_real
                        x_last_normal = x_0
                    y_attack = y_real - exp.attack_value
                else:
                    y_attack = y_real
                y_attack_arr.append(y_attack)

                if i == t_detect:
                    # x_0 estimate
                    print('-------------------------- x_0 estimate --------------------------')
                    attack_control = np.array([cin_arr[t_attack:t_detect]])
                    if len(x_last_normal.shape) == 0:
                        x_a = np.array([[x_last_normal]])
                    else:
                        x_a = x_last_normal #np.array([[item] for item in x_last_normal])
                    t1_start = time.time()
                    x0_lo, x0_up = est.estimate(x_a, attack_control)
                    time_info['x0_est'] = time.time() - t1_start
                    print('attack_control=', attack_control)
                    print("x_last_normal=", x_a)
                    print("x0_lo=", x0_lo)
                    print("x0_up=", x0_up)
                    print("x_0", x_0)
                    print('----------------------------------------------------')

                    # deadline estimate
                    print('-------------------------- deadline estimate --------------------------')
                    safe_set_lo = exp.safeset['lo']
                    safe_set_up = exp.safeset['up']
                    if exp.worst_case_control == 'current':
                        control = np.array([[cin_arr[-1]]])
                    elif exp.worst_case_control == 'control_up':
                        control = np.array([exp.control_limit['up']])
                    elif exp.worst_case_control == 'control_lo':
                        control = np.array([exp.control_limit['lo']])
                    elif exp.worst_case_control == 'zero':
                        control = np.array([[0]])
                    t2_start = time.time()
                    k = est.get_deadline(x0_lo, safe_set_lo, safe_set_up, control, max_k=exp.max_k)
                    time_info['ddl_est'] = time.time() - t2_start
                    recovered_index = i + k
                    print('k=', k)
                    print('----------------------------------------------------')

                    # recovery
                    print('-------------------------- recovery --------------------------')
                    # print(exp.sysd)
                    print('y_0=', x_0[exp.y_point])

                    target_set_lo = exp.target_set['lo']
                    target_set_up = exp.target_set['up']
                    control_lo = exp.control_limit['lo']
                    control_up = exp.control_limit['up']
                    print('target_set_lo=', target_set_lo[exp.y_point])
                    print('target_set_up=', target_set_up[exp.y_point])

                    r_control, time_info['formulate'], time_info['solve'] = recovery(exp.sysd, k, x0_lo, x0_up,
                                                                                     target_set_lo, target_set_up,
                                                                                     safe_set_lo, safe_set_up,
                                                                                     control_lo, control_up)
                    print("r_control=", r_control)
                    time_info['recovery_steps'] = k
                    time_info['total_time'] = time_info['x0_est'] + time_info['ddl_est'] + time_info['formulate'] + time_info['solve']
                    time_info['percentage'] = time_info['total_time'] / dt
                    time_info['k'] = k
                    time_statistics.append(time_info)
                    print('----------------------------------------------------')

                if t_detect <= i < t_detect + k:
                    j = i - t_detect
                    cin = r_control[0, j]
                    # pid.clear()
                else:
                    if i == t_detect + k:
                        print('recovered_state=', x_0[exp.y_point])
                    pid.SetPoint = ref[i]
                    pid.update(feedback_value=y_attack, current_time=i * dt)
                    cin = pid.output

                cin_arr.append(cin)
                yout, T, xout = lsim(sys, cin, [0, dt], x_0)
                y_real = yout[-1]
                if len(xout.shape) == 1:
                    x_0 = xout[-1]
                else:
                    x_0 = xout[-1, :].T
            y_arr_list.append(y_real_arr.copy())

            if exp.sep_graph:
                plt.figure()
                plt.plot(t_arr, y_real_arr)
                plt.plot(t_arr, ref)
                filename = os.path.join(rp, 'rt_recovery.jpg')
                plt.savefig(filename)

                plt.figure()
                plt.ylabel(exp.y_label)
                plt.plot(t_arr, ref, 'k--')
                plt.plot(t_arr, y_arr_list[0], 'r')
                plt.plot(t_arr, y_arr_list[1], 'y')
                plt.plot(t_arr, y_arr_list[2], 'b')
                filename = os.path.join(rp, 'all.jpg')
                plt.savefig(filename)

            plt.figure(figsize=(5.3, 1.7), dpi=300)
            plt.ylabel(exp.y_label)
            x_left = exp.x_left
            y_right = recovered_index * dt
            plt.plot(t_arr, ref, 'k--')
            plt.plot(t_arr, y_arr_list[0], 'r')
            plt.plot(t_arr, y_arr_list[1], 'y')
            plt.plot(t_arr, y_arr_list[2], 'b')
            plt.xlim(x_left, y_right)
            if not (exp.y_up is None or exp.y_lo is None):
                plt.ylim(exp.y_lo, exp.y_up)
            plt.tight_layout()
            filename = os.path.join(rp, 'final.jpg')
            plt.savefig(filename)

        except Exception as e:
            print('>>>>> ERROR - ', exp.name, ':', e)
            track = traceback.format_exc()
            print(track)

    for item in time_statistics:
        print(item)
