#!/usr/bin/env python3
'Load a roadmap file and print statistics'

import argparse
import sys
import os
import gzip
import json
import tempfile
import io

def populate_parser(parser=None):
    if not parser:
        parser = argparse.ArgumentParser()
    parser.description = '''
        Load a roadmap file and print statistics.
        Supported file types are *.dat and *.dat.gz.
        Note, this is slow for very large roadmaps.
        '''
    parser.add_argument('roadmaps', metavar='roadmap', nargs='+')
    parser.add_argument('-V', '--voxel-stats', action='store_true',
                        help='''Print stats on voxel objects too (slow).
                            Requires the cpptendon python library.
                            ''')
    return parser

def roadmap_parse(fname):
    '''Parses the roadmap and returns an iterator of json objects
    Supports .dat, .dat.gz, .json, and .json.gz
    '''
    if fname.endswith('.gz'):
        fin = gzip.open(fname)
    else:
        fin = open(fname)

    with fin:
        if '.dat' in fname:
            for line in fin:
                yield json.loads(line)
        elif '.json' in fname:
            roadmap = json.load(fin)['VoxelCachedLazyPRM_roadmap']
            if 'vertices' in roadmap:
                for vertex in roadmap['vertices']:
                    vertex['type'] = 'vertex'
                    yield vertex
            if 'edges' in roadmap:
                for edge in roadmap['edges']:
                    edge['type'] = 'edge'
                    yield edge
        else:
            raise ValueError(f'Unsupported file type for {fname}')

_tmp_file = None
def json_to_voxels(obj):
    from cpptendon.collision import VoxelOctree as VoxOct
    global _tmp_file
    
    _tmp_file.seek(0)
    _tmp_file.truncate()
    
    json.dump(obj, _tmp_file)
    _tmp_file.flush()
    return VoxOct.from_file(_tmp_file.name)

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)

    with tempfile.NamedTemporaryFile(mode='w+', suffix='.json') as tmp_file:
        global _tmp_file
        _tmp_file = tmp_file

        for roadmap in args.roadmaps:
            print(roadmap)

            # TODO: any other stats to print?
            num_vertices = 0
            num_edges = 0
            num_vertex_tips = 0
            num_vertex_voxels = 0
            num_edge_voxels = 0

            # occupied voxels and blocks if --voxel-stats given
            num_vertex_voxel_blocks = 0
            num_vertex_voxel_voxels = 0
            num_edge_voxel_blocks = 0
            num_edge_voxel_voxels = 0

            for obj in roadmap_parse(roadmap):
                if obj['type'] == 'vertex':
                    num_vertices += 1
                    if 'tip_pos' in obj:
                        num_vertex_tips += 1
                    if 'voxels' in obj:
                        num_vertex_voxels += 1
                        if args.voxel_stats:
                            vox = json_to_voxels(obj['voxels'])
                            num_vertex_voxel_blocks += vox.nblocks()
                            num_vertex_voxel_voxels += vox.ncells()
                    if num_vertices % 1000 == 0:
                        print(end='v')
                        sys.stdout.flush()
                elif obj['type'] == 'edge':
                    num_edges += 1
                    if 'voxels' in obj:
                        num_edge_voxels += 1
                        if args.voxel_stats:
                            vox = json_to_voxels(obj['voxels'])
                            num_edge_voxel_blocks += vox.nblocks()
                            num_edge_voxel_voxels += vox.ncells()
                    if num_edges % 1000 == 0:
                        print(end='e')
                        sys.stdout.flush()
                else:
                    raise ValueError(f'Unsupported json object: {obj}')

            print()
            print(f'  # vertices:             {num_vertices}')
            print(f'    - with tip_pos:       {num_vertex_tips} / {num_vertices}')
            print(f'    - with voxels:        {num_vertex_voxels} / {num_vertices}')
            if args.voxel_stats and num_vertex_voxels:
                print(f'    - voxel blocks:       {num_vertex_voxel_blocks}')
                print(f'    - voxel voxels:       {num_vertex_voxel_voxels}')
                print(f'    - avg voxel blocks:   {num_vertex_voxel_blocks / num_vertex_voxels}')
                print(f'    - avg voxel voxels:   {num_vertex_voxel_voxels / num_vertex_voxels}')
            print(f'  # edges:                {num_edges}')
            print(f'    - with voxels:        {num_edge_voxels} / {num_edges}')
            if args.voxel_stats and num_edge_voxels:
                print(f'    - voxel blocks:       {num_edge_voxel_blocks}')
                print(f'    - voxel voxels:       {num_edge_voxel_voxels}')
                print(f'    - avg voxel blocks:   {num_edge_voxel_blocks / num_edge_voxels}')
                print(f'    - avg voxel voxels:   {num_edge_voxel_voxels / num_edge_voxels}')
            print()

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
