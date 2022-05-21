#!/usr/bin/env python3
'''Simple script to run IK on a given set of desired tip positions'''

import argparse
import sys
import os
import csv

import numpy as np

from cpptendon.tendon import TendonRobot
from cpptendon.controller import Controller

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = '''
        Run IK based on Levenberg-Marquardt on a given set of tip positions.
        '''

    parser.add_argument('robot_toml')
    parser.add_argument('requested_ik')
    parser.add_argument('--max-iter', default=100,
            help='Maximum number of iterations for IK controller')
    parser.add_argument('-o', '--output', default='ik.csv',
            help='Output CSV file, one line per requested IK.')
    parser.add_argument('-t', '--tolerance', default=0.0005,
            help='stopping criteria for workspace error cost')
    parser.add_argument('--p-update-threshold', default=1e-6,
            help='stopping criteria for config update per iteration')
    parser.add_argument('--grad-descent-max-threshold', default=1e-9,
            help='stopping criteria for gradient descent step, max element')
    parser.add_argument('--fd-delta', default=1e-3,
            help='finite difference distance for approximate Jacobian')
    parser.add_argument('--mu-init', default=1.0,
            help='initial value of damping factor for Damped Least Squares')
    parser.add_argument('-q', '--quiet', action='store_true',
            help='suppress many of the print messages')

    return parser

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    print(args)

    robot = TendonRobot.from_toml(args.robot_toml)
    controller = Controller(robot)
    with open(args.requested_ik, 'r') as fin:
        reader = csv.DictReader(fin)
        tips_des = [
            np.array([float(row['tip_x']),
                      float(row['tip_y']),
                      float(row['tip_z'])]) for row in reader]

    cols = ['i', 'tip_error',
            'tip_x_des', 'tip_y_des', 'tip_z_des',
            'tip_x', 'tip_y', 'tip_z']
    config_cols = [f'tau_{i+1}' for i in range(len(robot.tendons))]
    if robot.enable_rotation:   config_cols.append('theta')
    if robot.enable_retraction: config_cols.append('s_start')
    cols.extend([f'{c}_i' for c in config_cols]) # start config
    cols.extend(config_cols)
    cols.extend(['iters', 'fk_calls'])

    with open(args.output, 'w') as fout:
        writer = csv.DictWriter(fout, cols)
        writer.writeheader()

        for i, tip in enumerate(tips_des):
            x0 = robot.random_state()
            ik_res = controller.inverse_kinematics(x0, tip,
                    max_iters=args.max_iter,
                    mu_init=args.mu_init,
                    stop_threshold_JT_err_inf=args.grad_descent_max_threshold,
                    stop_threshold_Dp=args.p_update_threshold,
                    stop_threshold_err=args.tolerance,
                    finite_difference_delta=args.fd_delta,
                    verbose=not args.quiet,
                    normalize=False)
            row = {
                'i': i+1,
                'tip_error': ik_res.error,
                'tip_x_des': tip[0],
                'tip_y_des': tip[1],
                'tip_z_des': tip[2],
                'tip_x': ik_res.tip[0],
                'tip_y': ik_res.tip[1],
                'tip_z': ik_res.tip[2],
                'iters': ik_res.iters,
                'fk_calls': ik_res.num_fk_calls,
            }
            for j, col in enumerate(config_cols):
                row[col] = x0[j]
                row[f'{col}_i'] = ik_res.state[j]

            writer.writerow(row)


    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
