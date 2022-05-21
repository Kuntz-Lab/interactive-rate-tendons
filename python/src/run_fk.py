#!/usr/bin/env python3
'''Simple script to run IK on a given set of desired tip positions'''

import argparse
import sys
import os
import csv

import numpy as np

from cpptendon.tendon import TendonRobot
from cpptendon.motion_planning import Problem

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = '''
        Run IK based on Levenberg-Marquardt on a given set of tip positions.
        '''

    parser.add_argument('robot_toml')
    parser.add_argument('config_csv')
    parser.add_argument('-o', '--output', default='fk.csv',
            help='''Output CSV file, one line per requested FK.
                The file does not have a header, and will simply have a format
                of x0,y0,z0,x1,y1,z1,...,xn,yn,zn.  Each row may have a
                different number of points than the last row, which is why
                there is no header.
                ''')
    parser.add_argument('-q', '--quiet', action='store_true',
            help='suppress many of the print messages')

    return parser

def _flatten_points(pts):
    for pt in pts:
        for val in pt:
            yield val

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    print(args)

    robot = TendonRobot.from_toml(args.robot_toml)
    print(f'reading {args.config_csv}')
    configs = Problem.load_plan(args.config_csv)
    print(f'writing {args.output}')
    with open(args.output, 'w') as fout:
        writer = csv.writer(fout)
        writer.writerows(_flatten_points(robot.forward_kinematics(conf))
                         for conf in configs)

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
