#!/usr/bin/env python3

# blender [<from-blend-file>] --background --python waypoints.py -- <points-csv>

# Example code came from
#   https://behreajj.medium.com/scripting-curves-in-blender-with-python-c487097efd13

import argparse
import csv
import os
import sys

import blender_helpers as helpers

workdir = os.path.dirname(os.path.abspath(__file__))

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
        Creates a blend file with the given waypoints as a path.
        This script can be called directly, in which case it will forward the call to blender like so:
            blender --background
                [<blend-file>]
                [<blender-args> ...]
                --python {__file__}
                --
                <script-args>
        '''
    parser.add_argument('points_csv')
    parser.add_argument('-s', '--scale', type=float, default=100.0,
            help='Scale points before making curves')
    parser.add_argument('-b', '--base', default='tendon-base',
            help='Base object to duplicate along paths')
    parser.add_argument('-o', '--output', default='autogen.blend',
            help='Output blender file')

    # if called directly, add blender args only for --help to work nicely
    if bpy is None:
        helpers.populate_blender_args(parser)

    return parser

def grouper(iterable, n):
    '''Groups a flat iterable into an iterable of n-tuples
    >>> list(grouper([1, 2, 3, 4, 5, 6, 7, 8], 2))
    [(1, 2), (3, 4), (5, 6), (7, 8)]
    >>> list(grouper([1, 2, 3, 4, 5, 6, 7, 8, 9], 3))
    [(1, 2, 3), (4, 5, 6), (7, 8, 9)]
    '''
    return zip(*[iter(iterable)]*n)

_collections = {}
def _get_collection(name):
    if name not in _collections:
        _collections[name] = bpy.context.blend_data.collections.new(name=name)
        bpy.context.collection.children.link(_collections[name])
    return _collections[name]

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)

    if bpy is None:
        helpers.call_through_blender(arguments, __file__)
        return

    # TODO: put paths in one collection and tendons in another
    base = None
    if args.base in bpy.context.scene.objects:
        base = bpy.context.scene.objects[args.base]
    elif args.base:
        print(f'Warning: base object {args.base} not found')

    cap_name = 'tendon-end'
    sphere_end = None
    if cap_name in bpy.context.scene.objects:
        sphere_end = bpy.context.scene.objects['tendon-end']

    with open(args.points_csv, 'r') as fin:
        reader = csv.reader(fin)
        shapes = [[Vector(pt) for pt in
                   grouper((args.scale * float(x) for x in row), 3)]
                  for row in reader]

        d = 1.5 * max((a - b).magnitude for shape in shapes
                      for a, b in zip(shape[:-1], shape[1:]))

        for i, pts in enumerate(shapes):
            print('adding curve', i)
            ops.curve.primitive_bezier_curve_add(enter_editmode=True)
            ops.curve.subdivide(number_cuts=len(pts)-2)

            curve = bpy.context.active_object
            _get_collection('paths').objects.link(curve)
            bpy.context.collection.objects.unlink(curve)
            spline = curve.data.splines[-1]
            bezier_points = spline.bezier_points
            for knot, pt in zip(bezier_points, pts):
                knot.co = pt
                # handle types:
                # - 'AUTO': smooth
                # - 'VECTOR': piecewise linear
                # - 'FREE': completely free
                knot.handle_left_type  = 'AUTO'
                knot.handle_right_type = 'AUTO'
                #knot.handle_left_type  = 'VECTOR'
                #knot.handle_right_type = 'VECTOR'

            ops.object.mode_set(mode='OBJECT')

            if base:
                path_obj = base.copy()
                path_obj.data = base.data.copy()
                path_obj.hide_render = False
                _get_collection('tendons').objects.link(path_obj)

                array_mod = path_obj.modifiers.new(name='Array', type='ARRAY')
                curve_mod = path_obj.modifiers.new(name='Curve', type='CURVE')

                array_mod.fit_type = 'FIT_CURVE'
                array_mod.curve = curve
                array_mod.use_relative_offset = True
                array_mod.relative_offset_displace = (0.0, 0.0, 1.0)
                array_mod.merge_threshold = d / 10
                array_mod.use_merge_vertices = True
                if sphere_end:
                    array_mod.end_cap = sphere_end

                curve_mod.object = curve
                curve_mod.deform_axis = 'POS_Z'

    print(f'Creating {args.output}')
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(workdir, args.output))

if __name__ == '__main__':
    main(helpers.extract_blender_script_args(sys.argv[1:]))
