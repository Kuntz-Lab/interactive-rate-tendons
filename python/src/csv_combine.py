#!/usr/bin/env python3

'Combines multiple CSVs into one'

import argparse
import csv
import sys

def parse_args(arguments):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description='Combines multiple CSVs into one.')
    parser.add_argument('csvfiles', metavar='csv', nargs='+',
                        help='''
                            List of CSV files to combine.  Each CSV file must
                            have the exact same set of columns.  The ordering
                            of columns of the first file is used.  The rows
                            will be output from the first file, then second,
                            etc.
                            ''')
    parser.add_argument('-o', '--out', default='combined.csv',
                        help='Where to output the combination')
    return parser.parse_args(arguments)

def all_have_same_header_rows(csvfiles):
    for csvfile in csvfiles:
        first = True
        with open(csvfile, 'r') as csvin:
            reader = csv.DictReader(csvin)
            if first:
                first_header = set(reader.fieldnames)
            elif set(reader.fieldnames) == first_header:
                return False
    return True

def main(arguments):
    args = parse_args(arguments)

    if not all_have_same_header_rows(args.csvfiles):
        print('All given CSV files must have the same header row',
              file=sys.stderr)
        return 1

    # get header row
    with open(args.csvfiles[0], 'r') as fin:
        reader = csv.DictReader(fin)
        header = reader.fieldnames

    with open(args.out, 'w') as fout:
        writer = csv.DictWriter(fout, header)
        writer.writeheader()
        for csvfile in args.csvfiles:
            print(f'{csvfile} >> {args.out}')
            with open(csvfile, 'r') as fin:
                writer.writerows(csv.DictReader(fin))

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
