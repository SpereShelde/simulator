import os
from control.matlab import lsim
from lib.PID import PID
from experiment import vehicle_turning, RLC_circuit, dc_motor_position, aircraft_pitch, quadrotor
from lib.recovery import recovery
import matplotlib.pyplot as plt
import numpy as np
import time

if __name__ == '__main__':
    exp_list = [vehicle_turning, RLC_circuit, dc_motor_position, aircraft_pitch, quadrotor]
    exps = []
    print('Benchmark List: \n  1. Vehicle Turning\n  2. RLC Circuit\n  3. DC Motor Position\n'
          '  4. Aircraft Pitch\n  5. Quadrotor\n  6. All of above')
    benchmark = int(input('Please choose the benchmark: (1-6) '))
    if 1 <= benchmark <= 5:
        exps.append(exp_list[benchmark-1])
    else:
        exps = exp_list

    attack_list = ['modification', 'delay', 'replay']
    attacks = []
    print('Attack List: \n  1. Modification\n  2. Delay\n  3. Replay\n  4. All of above')
    attack_num = int(input('Please choose the attack: (1-4) '))
    if 1 <= attack_num <= 3:
        attacks.append(attack_list[attack_num-1])
    else:
        attacks = attack_list

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
        func = exp.attacks['modification']['func']
        delay_step = exp.attacks['delay']['step']
        replay_start = exp.attacks['replay']['start']
        replay_end = exp.attacks['replay']['end']

        # print('----------------------', attack, ' attack w/o recovery   --------------------')
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

        for attack in attacks:
            try:
                y_arr_list = []
                time_info = {'exp': exp.name, 'attack': attack}

                # print('----------------------', attack, ' attack w/o recovery   --------------------')
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
                        if attack == 'modification':  # modification
                            y_attack = func(y_real)
                        elif attack == 'delay':  # delay
                            if i - t_attack > delay_step:
                                y_attack = y_real_arr[-delay_step]
                            else:
                                y_attack = y_last_normal
                        elif attack == 'replay':  # replay
                            delta_i = i - t_attack
                            j = replay_start + delta_i % (replay_end - replay_start)
                            y_attack = y_real_arr[j]
                        else:
                            y_attack = y_real
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

                # print('--------------------', attack, ' attack w/ non-real time recovery  ---------------------')
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
                        if attack == 'modification':  # modification
                            y_attack = func(y_real)
                        elif attack == 'delay':  # delay
                            if i - t_attack > delay_step:
                                y_attack = y_real_arr[-delay_step]
                            else:
                                y_attack = y_last_normal
                        elif attack == 'replay':  # replay
                            delta_i = i - t_attack
                            j = replay_start + delta_i % (replay_end - replay_start)
                            y_attack = y_real_arr[j]
                        else:
                            y_attack = y_real
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

                print('-------------------- ', attack, ' attack w/ real-time recovery  --------------------')
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
                        if attack == 'modification':  # modification
                            y_attack = func(y_real)
                        elif attack == 'delay':  # delay
                            if i - t_attack > delay_step:
                                y_attack = y_real_arr[-delay_step]
                            else:
                                y_attack = y_last_normal
                        elif attack == 'replay':  # replay
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
                        print('-------------------------- x_0 estimate --------------------------')
                        attack_control = np.array([cin_arr[t_attack:t_detect]])
                        if len(x_last_normal.shape) == 0:
                            x_a = np.array([[x_last_normal]])
                        else:
                            x_a = np.array([[item] for item in x_last_normal])
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
                        control = np.array([[0]])
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
                        print(exp.sysd)
                        if exp.k:
                            k = exp.k[attack]
                        target_set_lo = exp.target_set['lo']
                        target_set_up = exp.target_set['up']
                        control_lo = exp.control_limit['lo']
                        control_up = exp.control_limit['up']
                        r_control, time_info['formulate'], time_info['solve'] = recovery(exp.sysd, k, x0_lo, x0_up,
                                                                                         target_set_lo, target_set_up,
                                                                                         safe_set_lo, safe_set_up,
                                                                                         control_lo, control_up)
                        print("r_control=", r_control)
                        time_info['recovery_steps'] = k
                        time_info['total_time'] = time_info['x0_est'] + time_info['ddl_est'] + time_info['formulate'] + \
                                                  time_info['solve']
                        time_info['percentage'] = time_info['total_time'] / dt
                        time_statistics.append(time_info)
                        print('----------------------------------------------------')

                    if t_detect <= i < t_detect + k:
                        j = i - t_detect
                        cin = r_control[0, j]
                        # pid.clear()
                    else:
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
                filename = os.path.join(rp, attack + '.jpg')
                plt.savefig(filename)

            except Exception as e:
                print('>>>>> ERROR - ', exp.name, ':', e)

    for item in time_statistics:
        print(item)
