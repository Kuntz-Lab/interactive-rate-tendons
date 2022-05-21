#!/usr/bin/env python3

# blender --background --python decimate_mesh.py -- <fname> [...]

import argparse
import os
import sys
import time

import blender_helpers as helpers

# If we are called without being wrapped in Blender, call myself again
try:
    import bpy
    from bpy import ops
    from mathutils import Vector
except ModuleNotFoundError:
    bpy = None

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = '''
        Applies the decimate modifier from Blender onto an STL file and saves
        the result.

        This script can be called directly, in which case it will forward the call to Blender like so:
            blender --background --python {__file__} -- <script-args>
        '''
    parser.add_argument('stl_files', metavar='STL_FILE', nargs='+')

    if bpy is None:
        helpers.populate_blender_args(parser)

    return parser

def clean_objects():
    for item in bpy.data.objects:
        bpy.data.objects.remove(item)

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)

    if bpy is None:
        helpers.call_through_blender(arguments, __file__)
        return

    print()
    for fname in args.stl_files:
        start = time.time()

        clean_objects()
        print(f'{fname}')
        print('  loading: ', end='')
        bpy.ops.import_mesh.stl(filepath=fname)
        obj = bpy.context.object

        print('  stats:')
        mesh = obj.to_mesh()
        print(f'    verts: {len(mesh.vertices)}')
        print(f'    edges: {len(mesh.edges)}')
        obj.to_mesh_clear()

        print('  applying DECIMATE modifier')
        modifier = obj.modifiers.new(name='decimate', type='DECIMATE')
        modifier.decimate_type = 'DISSOLVE'
        bpy.ops.object.modifier_apply(modifier='decimate')

        print('  stats:')
        mesh = obj.to_mesh()
        print(f'    verts: {len(mesh.vertices)}')
        print(f'    edges: {len(mesh.edges)}')
        obj.to_mesh_clear()

        new_fname = os.path.splitext(fname)[0] + '-decimate.stl'
        print(f'  saving to {new_fname}')
        bpy.ops.export_mesh.stl(filepath=new_fname, use_selection=True)

        secs = time.time() - start
        print(f'took {secs} seconds')
        print()

if __name__ == '__main__':
    main(helpers.extract_blender_script_args(sys.argv[1:]))
