import argparse
import shutil
import sys
import os

__my_dir = os.path.dirname(os.path.abspath(__file__))

def populate_blender_args(parser=None):
    '''
    Argument to enable getting info to call this again through blender.
    '''
    if parser is None:
        parser = argparse.ArgumentParser()
        parser.description = 'Arguments for forwarding the call to blender'
    parser.add_argument('--blender-exe', default=shutil.which('blender'),
            help='Blender executable.')
    parser.add_argument('--blender-args',
            help='args to pass to blender as a comma-separated list')
    parser.add_argument('--blend-file', default=None,
            help='base blend file to augment')
    parser.add_argument('--no-blend-file', dest='blend_file',
            action='store_const', const=None,
            help='do not use a base blender file')
    return parser

def call_through_blender(arguments, script):
    '''
    Call the given python script through blender in python mode.
    '''
    import subprocess as subp
    parser = populate_blender_args()
    args, remaining_arguments = parser.parse_known_args(arguments)

    command = [args.blender_exe, '--background']
    if args.blend_file:     command.append(args.blend_file)
    if args.blender_args:   command.extend(args.blender_args.split(','))
    command.extend(['--python', script, '--'])
    command.extend(remaining_arguments)
    env = os.environ.copy()
    if 'PYTHONPATH' in env:
        env['PYTHONPATH'] += f':{__my_dir}'
    else:
        env['PYTHONPATH'] = __my_dir
    subp.call(command, env=env)

def extract_blender_script_args(arguments=sys.argv[1:]):
    '''
    Extract the command-line arguments related to the script, removing
    blender-specific ones (if any)
    '''
    try:
        import bpy
    except:
        return arguments
    else:
        if '--' in arguments:
            return arguments[arguments.index('--') + 1 : ]
        return []
