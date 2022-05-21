#!/usr/bin/env python3
'''Aggregate stats after plot_roadmap_chain_histogram.py'''

import argparse
import sys
import csv
import glob
import os
from collections import defaultdict

#import pandas as pd


def populate_parser(parser=None):
    '''Populate parser'''
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = '''
        Aggregate stats after plot_roadmap_chain_histogram.py.
        Will output a few files into the same directory as --directory.
          (a) all-stats.csv: accumulated statistics,
          (b) wpt-data.csv: full dataset, one waypoint per line,
          (c) tot-data.csv: data on each run, each run results in one row.
        '''
    parser.add_argument('-d', '--directory', default='.',
                        help='''base directory (default=".").
                            Should be structured like 05b-ablation/comparison/
                            ''')
    return parser

def gen_stats_data(indir, outfile):
    '''Gen stats csv (as <outfile>) from nested <indir>/**/stats.csv'''
    statfiles = sorted(glob.glob(os.path.join(indir, '**', 'stats.csv'), recursive=True))
    print(statfiles)
    print(f'Aggregating stats from {len(statfiles)} stats.csv files')

    configs = [[x] + _split_dir_fields(x) for x in statfiles]
    config_fields = ['file', 'filenum'] \
                  + [f'param{i+1}' for i in range(max(len(x) for x in configs)-1)]
    #config_fields = [
    #    'directory',
    #    'pts',
    #    'start',
    #    't',
    #    'planner',
    #    'sampling',
    #    'roadmap-precompute',
    #    'roadmap-loading',
    #    'ik-seq-vs-par',
    #    'ik-fast-vs-accu',
    #    'ik-lazy-vs-precompute',
    #]

    #print(config_fields)
    #for c in configs:
    #    print(c)

    cols = ['max', 'mean', 'total']
    fields = [
        'calls:fk',
        'time:fk-total',
        'time:collision-total',
        'time:roadmapIk',
        'time:solveWithRoadmap',
        'solution:tip-error',
        'solution:cost',
        'time:milestone',
        'time:ik-total',
    ]
    field_combos = [f'{f}_{c}' for f in fields for c in cols]
    fieldvals = []
    for infile in statfiles:
        row = defaultdict(lambda: defaultdict(str))
        print(f'reading {infile}')
        with open(infile, 'r') as fin:
            reader = csv.DictReader(fin)
            for csvrow in reader:
                if csvrow['name'] in fields:
                    row[csvrow["name"]] = csvrow
            fieldvals.append([row[f][c] for f in fields for c in cols])

    all_fields = config_fields + field_combos
    all_data = [conf + fvals for conf, fvals in zip(configs, fieldvals)]

    #df = pd.DataFrame(data=all_data, columns=all_fields)
    print(f'writing {outfile}')
    #df.to_csv(outfile)
    with open(outfile, 'w') as fout:
        writer = csv.writer(fout)
        writer.writerow(all_fields)
        writer.writerows(all_data)

def gen_wpt_data(indir, outfile):
    '''Gen waypoint data (as <outfile>) from nested <indir>/**/log.csv
    Auto-aggregates all fields of waypoints.  For fields present in one
    run or milestone and not another, those fields, will be empty.

    Fields in the output file will be sorted.
    '''
    logfiles = sorted(glob.glob(os.path.join(indir, '**', 'log.csv'), recursive=True))
    print(f'Aggregating all waypoint data from {len(logfiles)} log.csv files')

    configs = [[x, i] + _split_dir_fields(x) for i, x in enumerate(logfiles)]
    config_fields = ['file', 'filenum'] \
                  + [f'param{i+1}' for i in range(max(len(x) for x in configs)-2)]

    extra_fields = set()
    data_fields = ['milestone', 'file', 'filenum']
    data = []
    for conf in configs:
        conf = dict(zip(config_fields, conf)) # convert to dict
        infile = conf['file']
        # populate extra_fields and data
        print(f'reading {infile}')
        with open(infile, 'r') as fin:
            reader = csv.reader(fin)
            header = next(reader)
            assert len(header) >= 3
            assert header[:3] == ['name', 'milestone', 'value']
            start_milestone = 'start-milestone'
            end_milestone = 'time:milestone'
            num_milestones = 0
            in_milestone = False
            datum = None
            for row in reader:
                if row[0] == start_milestone:
                    num_milestones += 1
                    in_milestone = True
                    datum = defaultdict(str) # defaults an empty string value
                    data.append(datum) # append, then populate
                    datum.update(conf) # populate with config values
                    datum['milestone'] = num_milestones

                # handle a row within a milestone
                assert (not in_milestone) or (row[1] == str(num_milestones)), str(row)
                if in_milestone and row[0] != start_milestone:
                    extra_fields.add(row[0])
                    # if already there, empty it
                    if row[0] in datum and datum[row[0]] != row[2]:
                        datum[row[0]] = ''
                    else:
                        datum[row[0]] = row[2]

                if row[0] == end_milestone:
                    in_milestone = False

    assert all(x not in extra_fields for x in data_fields)

    print()
    print(f'writing {outfile}')
    with open(outfile, 'w') as fout:
        fields = config_fields + data_fields + sorted(extra_fields)
        writer = csv.DictWriter(fout, fields)
        writer.writeheader()
        writer.writerows(data)
    print()

def gen_tot_data(indir, outfile):
    '''Gen total data (as <outfile>) from nested <indir>/**/log.csv'''
    logfiles = sorted(glob.glob(os.path.join(indir, '**', 'log.csv'), recursive=True))
    print(f'Aggregating totals data from {len(logfiles)} log.csv files')

    configs = [[x, i] + _split_dir_fields(x) for i, x in enumerate(logfiles)]
    config_fields = ['file', 'filenum'] \
                  + [f'param{i+1}' for i in range(max(len(x) for x in configs)-2)]

    extra_fields = set()
    data_fields = ['milestone', 'file', 'filenum']
    data = []
    for conf in configs:
        dconf = dict(zip(config_fields, conf))
        infile = dconf['file']
        # populate extra_fields and data
        print(f'reading {infile}')
        with open(infile, 'r') as fin:
            reader = csv.reader(fin)
            header = next(reader)
            assert len(header) >= 3
            assert header[:3] == ['name', 'milestone', 'value']
            start_milestone = 'start-milestone'
            end_milestone = 'time:milestone'
            in_milestone = False
            datum = defaultdict(str) # defaults an empty string value
            data.append(datum) # append, then populate
            datum.update(dconf)
            for row in reader:
                if row[0] == start_milestone:
                    in_milestone = True

                # handle a row without a milestone
                if not in_milestone:
                    extra_fields.add(row[0])
                    assert row[1] == 'N/A'
                    # if already there, empty it
                    datum[row[0]] = '' if row[0] in datum else row[2]

                if row[0] == end_milestone:
                    in_milestone = False

    assert all(x not in extra_fields for x in data_fields)

    print()
    print(f'writing {outfile}')
    with open(outfile, 'w') as fout:
        fields = config_fields + data_fields + sorted(extra_fields)
        writer = csv.DictWriter(fout, fields)
        writer.writeheader()
        writer.writerows(data)
    print()

def _split_dir_fields(path):
    '''
    Split the separate components of the directory into pieces, separated by /
    and -
    '''
    apath = os.path.abspath(path)
    psplit = list(reversed([x for x in apath.split('/') if x])) # non-empty pieces
    if 'comparison' in psplit:
        i = psplit.index('comparison')
        assert i >= 2
        d1 = psplit[i-1]
        d2 = psplit[i-2]
    elif psplit[0] == 'log.csv':
        d1 = psplit[2]
        d2 = psplit[1]
    elif psplit[0] == 'stats.csv':
        d1 = psplit[3]
        d2 = psplit[2]
    else:
        raise ValueError(f'Unknown path given: {path}')

    return d1.split('-') + d2.split('-')

def main(arguments):
    'command-line entry point'
    parser = populate_parser()
    args = parser.parse_args(arguments)

    os.chdir(args.directory)
    gen_stats_data('.', 'all-stats.csv')
    gen_wpt_data('.', 'wpt-data.csv')
    gen_tot_data('.', 'tot-data.csv')

if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except Exception as ex:
        print('Unhandled exception:', file=sys.stderr)
        print(ex, file=sys.stderr, flush=True)
        raise
    else:
        sys.exit(0)
