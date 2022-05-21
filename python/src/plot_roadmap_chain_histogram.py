#!/usr/bin/env python3
'Plot histograms from the log file generated from roadmap_chained_plan'

import argparse
import csv
import sys
import os
from collections import defaultdict
import multiprocessing as mp

import numpy as np
import matplotlib
matplotlib.use('Agg') # no plotting windows, must be before import pyplot
import matplotlib.pyplot as plt

def populate_parser(parser=None):
    'Create an argument parser for this script'
    if not parser:
        parser = argparse.ArgumentParser()
    parser.description = '''Plot histograms from the log file generated from
                            roadmap_chained_plan'''
    parser.add_argument('-o', '--outdir', default='plots',
                        help='''
                            Where to output the plots.  Default is ./plots/
                            ''')
    parser.add_argument('log',
                        help='''
                            logfile to parse.  It is the logfile output from
                            roadmap_chained_plan.
                            ''')
    parser.add_argument('-j', '--jobs', type=int, default=mp.cpu_count(),
                        help='''
                            Parallelize with this number of jobs.
                            Defaults to the number of CPUs.
                        ''')
    parser.add_argument('--all-events', dest='event_type', action='store_const',
                        const='all', default='critical',
                        help='Make histograms for all events, not just a few')
    parser.add_argument('--main-events', dest='event_type',
                        action='store_const', const='main', default='critical',
                        help='Make histograms for main events, not just a few')
    parser.add_argument('--critical-events', dest='event_type',
                        action='store_const', const='critical',
                        default='critical',
                        help='''
                            Make histograms for only the critical events.  This
                            is the default behavior.
                            ''')
    parser.add_argument('--bins', type=int, default=40,
                        help='''
                            number of bins to use in the histogram (default =
                            40)
                            ''')
    parser.add_argument('--noplot', action='store_true',
                        help='no plotting, just generate stats.csv')
    return parser

def mkdir_p(directory):
    'The same as calling mkdir -p <directory> on the command-line'
    os.makedirs(directory, exist_ok=True)

def plot_histogram(fname, event_type, vals, max_bins):
    '''
    Plot histogram to the given filename using the given event type and the
    values provided.

    @param fname (str): name of file to output (supported by matplotlib)
    @param event_type (str): type of event
    @param vals (list(float)): values to bin into a histogram
    @param max_bins (int): maximum number of bins to use.  If the event_type
        starts with 'calls:', then it will interpret the vals as integers and
        determine if the integer range is less than the max_bins.  If it is,
        then that range is used instead so that the histogram looks continuous.
        Otherwise, the number of bins will be set to the max_bins.
    '''
    if not vals: # only plot if we have something to plot
        print(f'Warning: vals is empty for {event_type}, skipping histogram')
        return

    if len(vals) < 2:
        print(f'Warning: vals has only one value for {event_type}, skipping histogram')
        return

    if ':' in event_type:
        measure, event_name = event_type.split(':', 1)
    else:
        measure = 'time'
        event_name = event_type

    # determine the number of bins
    if measure == 'calls':
        vals = [int(x) for x in vals]
        spread = 1 + max(vals) - min(vals)
        bins = min(max_bins, spread)
    else:
        bins = max_bins

    plt.cla()
    plt.clf()

    if measure == 'time':
        plt.xlabel('seconds')
    elif measure == 'solution':
        plt.xlabel(event_name)
    else:
        plt.xlabel(measure)

    plt.title(event_type)
    plt.ylabel('count')
    plt.hist(vals, bins=bins)
    print('saving', fname)
    plt.savefig(fname)

def plot_per_milestone(fname, event_type, vals):
    '''
    Plot the values on the y-axis as a sequence along waypoint count on the
    bottom.  The waypoints are assumed to start at 1 for the first value and
    increment by one.
    '''
    if not vals: # only plot if we have something to plot
        print(f'Warning: vals is empty for {event_type}, skipping plot')
        return

    if len(vals) < 2:
        print(f'Warning: vals has only one value for {event_type}, skipping plot')
        return

    if ':' in event_type:
        measure, event_name = event_type.split(':', 1)
    else:
        measure = 'time'
        event_name = event_type

    plt.cla()
    plt.clf()

    if measure == 'time':
        plt.ylabel('Time (s)')
    elif measure == 'solution':
        plt.ylabel(event_name)
    else:
        plt.ylabel(measure)

    plt.title(event_type)
    plt.xlabel('milestone')
    plt.plot(vals)
    print('saving', fname)
    plt.savefig(fname)

def handle_event_vals(event_type, vals, outdir, bins):
    '''Create dat file and histogram plot of vals in outdir'''
    datname = os.path.join(outdir, event_type + '.dat')
    with open(datname, 'w') as datout:
        for val in vals:
            print(val, file=datout)
    fname = os.path.join(outdir, 'hist-' + event_type.replace(':', '__') + '.png')
    try:
        plot_histogram(fname, event_type, vals, bins)
    except ValueError as ex:
        print(f'Warning ({fname}): {ex}')
    fname = os.path.join(outdir, 'plot-' + event_type.replace(':', '__') + '.png')
    try:
        plot_per_milestone(fname, event_type, vals)
    except ValueError as ex:
        print(f'Warning ({fname}): {ex}')

def main(arguments):
    'Main logic here'
    parser = populate_parser()
    args = parser.parse_args(arguments)

    all_events = [
        'calls:astar',
        'calls:collision',
        'calls:collision-swept-volume',
        'calls:collision-without-voxelizing',
        'calls:fk',
        'calls:reindex',
        'calls:remove_vertices',
        'calls:self_collision',
        'calls:voxelize',
        'calls:voxelize-swept-volume',
        'ik:final-error',
        'ik:restart-count',
        'solution:cost',
        'solution:tip-error',
        'solution:waypoints',
        'time:astar-max',
        'time:astar-mean',
        'time:astar-median',
        'time:astar-min',
        'time:astar-total',
        'time:collision-max',
        'time:collision-mean',
        'time:collision-median',
        'time:collision-min',
        'time:collision-swept-volume-max',
        'time:collision-swept-volume-mean',
        'time:collision-swept-volume-median',
        'time:collision-swept-volume-min',
        'time:collision-swept-volume-total',
        'time:collision-total',
        'time:collision-without-voxelizing-max',
        'time:collision-without-voxelizing-mean',
        'time:collision-without-voxelizing-median',
        'time:collision-without-voxelizing-min',
        'time:collision-without-voxelizing-total',
        'time:fk-max',
        'time:fk-mean',
        'time:fk-median',
        'time:fk-min',
        'time:fk-total',
        'time:ik',
        'time:ik-fix',
        'time:ik-fix-max',
        'time:ik-fix-mean',
        'time:ik-fix-median',
        'time:ik-fix-min',
        'time:ik-fix-total',
        'time:ik-max',
        'time:ik-mean',
        'time:ik-median',
        'time:ik-min',
        'time:ik-restarts-max',
        'time:ik-restarts-mean',
        'time:ik-restarts-median',
        'time:ik-restarts-min',
        'time:ik-restarts-total',
        'time:ik-total',
        'time:ik-with-restarts',
        'time:ik_with_restarts',
        'time:milestone',
        'time:milestone-max',
        'time:milestone-mean',
        'time:milestone-median',
        'time:milestone-min',
        'time:milestone-total',
        'time:reindex-max',
        'time:reindex-mean',
        'time:reindex-median',
        'time:reindex-min',
        'time:reindex-total',
        'time:roadmapIk',
        'time:self_collision-max',
        'time:self_collision-mean',
        'time:self_collision-median',
        'time:self_collision-min',
        'time:self_collision-total',
        'time:solve',
        'time:solve-max',
        'time:solve-mean',
        'time:solve-median',
        'time:solve-min',
        'time:solve-total',
        'time:solveWithRoadmap',
        'time:voxelize-max',
        'time:voxelize-mean',
        'time:voxelize-median',
        'time:voxelize-min',
        'time:voxelize-swept-volume-max',
        'time:voxelize-swept-volume-mean',
        'time:voxelize-swept-volume-median',
        'time:voxelize-swept-volume-min',
        'time:voxelize-swept-volume-total',
        'time:voxelize-total',
        ]
    main_events = [
        'calls:fk',
        'solution:tip-error',
        'solution:waypoints',
        'time:astar-total',
        'time:collision-total',
        'time:collision-without-voxelizing-total',
        'time:fk-total',
        'time:ik_with_restarts',
        'time:milestone',
        'time:roadmapIk',
        'time:solve',
        'time:solveWithRoadmap',
        'time:voxelize-swept-volume-total',
        'time:voxelize-total',
        'ik:restart-count',
        ]
    critical_events = [
        'solution:tip-error',
        'time:ik_with_restarts',
        'time:milestone',
        'time:roadmapIk',
        'time:solve',
        'time:solveWithRoadmap',
        ]

    if args.event_type == 'all':
        events = all_events
    elif args.event_type == 'main':
        events = main_events
    elif args.event_type == 'critical':
        events = critical_events
    else:
        raise NotImplementedError('Unimplemented event type: ' + args.event_type)

    print('parsing', args.log)
    values = defaultdict(list)
    observed_events = set()
    with open(args.log, 'r') as fin:
        reader = csv.DictReader(fin)
        for row in reader:
            if row['name'] in events:
                observed_events.add(row['name'])
                values[row['name']].append(float(row['value']))

    mkdir_p(args.outdir)
    if not args.noplot:
        # generate plots and dat files
        params = sorted([(k, v, args.outdir, args.bins)
                         for k, v in values.items()])
        # This is slow, so we parallelized it
        if args.jobs > 1:
            print(f'{os.path.basename(__file__)}: (j={args.jobs}) plotting in parallel')
            with mp.Pool(args.jobs) as pool:
                pool.starmap(handle_event_vals, params)
        else:
            print(f'{os.path.basename(__file__)}: (j=1) plotting sequentially')
            for (k, v, outdir, bins) in params:
                handle_event_vals(k, v, outdir, bins)

    fname = os.path.join(args.outdir, 'stats.csv')
    print('creating', fname)
    with open(fname, 'w') as fout:
        header = ['name', 'count', 'min', 'mean', 'median', 'max', 'total']
        writer = csv.DictWriter(fout, header)
        writer.writeheader()
        for name, vals in values.items():
            row = {
                'name': name,
                'count': len(vals),
                'min': min(vals),
                'mean': np.mean(vals),
                'median': np.median(vals),
                'max': max(vals),
                'total': sum(vals),
                }
            writer.writerow(row)

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
