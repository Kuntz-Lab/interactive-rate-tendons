#!/usr/bin/env python3
'''
Perform operations reguarding voxel objects
'''

import argparse
import os
import sys
from abc import ABC, abstractmethod
import json

from roadmap_stats import roadmap_parse

from cpptendon.collision import VoxelOctree, Sphere, Capsule
from cpptendon.tendon import BackboneSpecs
from cpptendon.motion_planning import Environment

class AbstractSubcommand(ABC):
    '''
    Interface class for command-line subcommands.  Classes that derive from
    this class will automatically be included in the command-line parsing as
    well as being executed when that subcommand is requested.
    '''
    @property
    @abstractmethod
    def name(self):
        '''
        Subcommand name - to be passed in the command-line to request this
        subcommand
        '''

    @property
    @abstractmethod
    def brief_description(self):
        '''
        Return a string with a brief description to be used as the help
        documentation at the top-level call.  It should briefly describe what
        this subcommand does.
        '''

    @abstractmethod
    def populate_parser(self, parser=None):
        '''
        Populate and return the given parser from argparse.  This will be used
        in a top-level parser as a subparser.
        '''

    @abstractmethod
    def run(self, parsed_args):
        '''
        Perform the subcommand given the parsed arguments as returned by
        argparse.ArgumentParser.parse_args()

        Should return an integer that represents the return code for the
        program (i.e., 0 for success and any other value for error)
        '''

class VoxelizeEnvSubcommand(AbstractSubcommand):
    '''
    Voxelize an Environment specified in a toml file as a Voxel object.
    '''
    @property
    def name(self):
        return 'voxelize-env'

    @property
    def brief_description(self):
        return 'Voxelize the [Environment] section from a toml file into a voxel object.'

    def populate_parser(self, parser=None):
        if not parser:
            parser = argparse.ArgumentParser()
        parser.formatter_class = argparse.ArgumentDefaultsHelpFormatter
        parser.description = self.brief_description + '''
                Mesh objects in the [Environment] section are not currently
                supported.  An exception will be thrown if meshes are
                specified.
                '''
        parser.add_argument('toml',
                            help='''
                                Toml file containing an [Environment] section
                                ''')
        parser.add_argument('output', help='output filename for voxel object')

        limit_group = parser.add_mutually_exclusive_group(required=True)
        limit_group.add_argument('--limits', nargs=6, type=float,
                metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX', 'ZMIN', 'ZMAX'),
                help='''
                    Specify the six limits of the space.
                    This option conflicts with --limits-from-robot.
                    There is no default behavior, so either --limits or
                    --limits-from-robot need to be specified.
                    ''')
        limit_group.add_argument('--limits-from-robot',
                metavar='ROBOT_TOML',
                help='''
                    Set the voxel limits to the spherical bound of the
                    workspace around the robot.  If the maximum length of the
                    robot is L (taken from backbone_specs.length), then the
                    limits in x, y, and z are [-L, L].  This loads the robot
                    length from the given toml file and sets the limits using
                    that.
                    This option conflicts with --limits.
                    There is no default behavior, so either --limits or
                    --limits-from-robot need to be specified.
                    ''')

        parser.add_argument('-N', '--voxel-dim',
                choices=[4, 8, 16, 32, 64, 128, 256, 512],
                type=int,
                default=128,
                metavar='VOXEL_DIM',
                help='''
                    Number of voxels in each dimension.  This number will
                    discretize the x, y, and z dimensions by this many
                    segments.  The number needs to be a power of two and the
                    current available choices are {4, 8, 16, 32, 64, 128, 256,
                    512}.
                    ''')
        parser.add_argument('--dilate-environment', metavar='RADIUS', type=float,
                help='''
                    Rather than just voxelizing the environment, first increase
                    the size of the environment by the given radius, then
                    voxelize it.  So points become spheres, spheres and
                    capsules increase their radius.  This option is useful if
                    you only want to voxelize the robot backbone centerline for
                    collision checking, where you dilate the environment by the
                    robot radius.
                    ''')
        parser.add_argument('-p', '--workspace-padding-factor', type=float,
                default=0.05, help='''
                    Pad the generated workspace (if doing --limits-from-robot)
                    so that floating-point errors don't cause things to break.
                    ''')
        return parser

    def run(self, parsed_args):
        args = parsed_args

        # create an empty voxel object
        voxels = VoxelOctree(args.voxel_dim)
        if args.limits:
            voxels.xlim = (args.limits[0], args.limits[1])
            voxels.ylim = (args.limits[2], args.limits[3])
            voxels.zlim = (args.limits[4], args.limits[5])
        elif args.limits_from_robot:
            specs = BackboneSpecs.from_toml(args.limits_from_robot)
            L = specs.L * (1 + args.workspace_padding_factor)
            voxels.xlim = (-L, L)
            voxels.ylim = (-L, L)
            voxels.zlim = (-L, L)
        else:
            raise NotImplementedError('Unsupported limits option used')

        # load the environment and dilate
        env = Environment.from_toml(args.toml)
        dilate = 0.0
        if args.dilate_environment:
            dilate = args.dilate_environment
        voxels = env.voxelize(voxels, dilate)

        print('writing', args.output)
        voxels.to_file(args.output)

class VoxelDilateSubcommand(AbstractSubcommand):
    '''
    Dilate a VoxelOctree
    '''
    # unique objects for use in comparisons
    N6     = '6 neighbor'
    N27    = '27 neighbor'
    SPHERE = 'sphere'

    @property
    def name(self):
        return 'dilate'

    @property
    def brief_description(self):
        return 'Dilate a given VoxelOctree'

    def populate_parser(self, parser=None):
        if not parser:
            parser = argparse.ArgumentParser()
        parser.formatter_class = argparse.ArgumentDefaultsHelpFormatter
        parser.description = self.brief_description + '''
                Dilate a given VoxelOctree
                '''
        parser.add_argument('input', help='input voxel file')
        parser.add_argument('output', help='output voxel file')

        neighbor_group = parser.add_mutually_exclusive_group(required=False)
        neighbor_group.add_argument('-n6', '--6-neighbor', dest='type',
                action='store_const', const=self.N6, default=self.N6,
                help='Dilate one voxel with 6 neighbors.')
        neighbor_group.add_argument('-n27', '--27-neighbor', dest='type',
                action='store_const', const=self.N27, default=self.N6,
                help='Dilate one voxel with 27 neighbors.')
        neighbor_group.add_argument('-r', '--radius', dest='radius',
                type=float, default=None,
                help='''
                    Dilate by a sphere of the given radius in meters, not an
                    integer number of voxels.
                    ''')
        return parser

    def run(self, args):
        if isinstance(args.radius, float):
            args.type = self.SPHERE

        print('Performing dilation')
        print(f'  input:    {args.input}')
        print(f'  output:   {args.output}')
        print(f'  type:     {args.type}')
        if args.type is self.SPHERE:
            print(f'  radius:   {args.radius}')
        print()

        print(f'loading {args.input}')
        vox = VoxelOctree.from_file(args.input)

        print(f'dilating voxels ({args.type})')
        if   args.type is self.N6:     vox.dilate_6neighbor()
        elif args.type is self.N27:    vox.dilate_27neighbor()
        elif args.type is self.SPHERE: vox.dilate_sphere(args.radius)

        print(f'writing {args.output}')
        vox.to_file(args.output)

class ExtractFromRoadmapSubcommand(AbstractSubcommand):
    '''
    Extract from roadmap
    '''

    @property
    def name(self):
        return 'roadmap-extract'

    @property
    def brief_description(self):
        return 'Extract voxelized shapes from a roadmap'

    def populate_parser(self, parser=None):
        if not parser:
            parser = argparse.ArgumentParser()
        parser.formatter_class = argparse.ArgumentDefaultsHelpFormatter
        parser.description = self.brief_description + '''
                Extract all voxelized vertices and edges from a roadmap into
                separate files.
                '''
        parser.add_argument('roadmap', help='roadmap_file')
        parser.add_argument('outdir', default='extracted_from_roadmap',
                            help='output directory')
        return parser

    def run(self, args):
        print()
        print(f'Extracting from roadmap: "{args.roadmap}"')
        print()

        print(f'loading {args.roadmap}')
        roadmap_objects = roadmap_parse(args.roadmap)

        os.makedirs(args.outdir, exist_ok=True)
        for obj in roadmap_objects:
            if obj['type'] == 'vertex':
                if 'voxels' in obj:
                    fname = os.path.join(args.outdir, f'v{obj["index"]}.json')
                    print(f'  writing {fname}')
                    with open(fname, 'w') as fout:
                        json.dump(obj['voxels'], fout)
                        fout.write('\n')
            elif obj['type'] == 'edge':
                if 'voxels' in obj:
                    fname = os.path.join(args.outdir,
                        f'e-{obj["source"]}-{obj["target"]}.json')
                    print(f'  writing {fname}')
                    with open(fname, 'w') as fout:
                        json.dump(obj['voxels'], fout)
                        fout.write('\n')

class ToStlSubcommand(AbstractSubcommand):
    '''
    Convert voxel files to stl (much like nrrd2mesh)
    '''

    @property
    def name(self):
        return 'to-stl'

    @property
    def brief_description(self):
        return 'Convert voxel files to stl (much like nrrd2mesh)'

    def populate_parser(self, parser=None):
        if not parser:
            parser = argparse.ArgumentParser()
        parser.formatter_class = argparse.ArgumentDefaultsHelpFormatter
        parser.description = self.brief_description + '''
            For each input file, will replace the extension with .stl and write
            '''
        parser.add_argument('voxelfiles', metavar='voxelfile', nargs='+',
                            help='input files')
        parser.add_argument('-d', '--directory', default=None,
                            help='output to this directory instead')
        parser.add_argument('-a', '--ascii', action='store_true',
                            help='save in ascii format instead of binary.')
        return parser

    def run(self, args):
        print('Converting voxels to STL')
        print(f'  - directory: {args.directory}')
        print(f'  - ascii:     {args.ascii}')
        print()

        for fname in args.voxelfiles:
            newname = os.path.splitext(fname)[0] + '.stl'
            if args.directory:
                newname = os.path.join(directory, os.path.basename(newname))
            print(f'{fname} -> {newname}')

            print(f'  - loading {fname}: ', end='', flush=True)
            vox = VoxelOctree.from_file(fname)
            print('done')

            print('  - converting to mesh: ', end='', flush=True)
            mesh = vox.to_mesh()
            print('done')

            print(f'  - writing to {newname}: ', end='', flush=True)
            mesh.to_stl(newname, not args.ascii)
            print('done')

            print()


def all_subclasses(base_class, var_dict=None):
    '''
    Returns all classes in a given scope that are derived from the given base
    class.

    You can pass in the var_dict object that is a dictionary of all variables
    to search (e.g., globals() or locals()).  The default is to use globals()
    which is the global variables in the file where this function is defined.

    Returns a dictionary of name -> class
    '''
    if var_dict is None:
        var_dict = globals()
    return {key: value for key, value in var_dict.items()
            if isinstance(value, type)
            and value != base_class
            and issubclass(value, base_class)}

def generate_subcommands(var_dict=None):
    '''
    Finds the list of classes that derive from AbstractSubcommand from the
    given variable scope (as specified by var_dict).  Then it instantiates an
    instance of each one.

    @param var_dict: dictionary of variables to search for classes derived from
        AbstractSubcommand.  By default, this will be globals() which are the
        global variables defined within the file where this function resides.

    @return a dictionary of subcommand.name -> <subcommand instance>
    '''
    subcommand_dict = all_subclasses(AbstractSubcommand)
    subcommand_instances = [x() for x in subcommand_dict.values()]
    return {x.name: x for x in subcommand_instances}

def populate_parser(subcommand_map, parser=None):
    if not parser:
        parser = argparse.ArgumentParser(
            formatter_class=argparse.ArgumentDefaultsHelpFormatter,
            )
    parser.description = '''
        Performs various voxel operations.
        '''
    subparsers = parser.add_subparsers(
            title='subcommands',
            dest='subcommand',
            metavar='subcommand',
            )
    subparsers.required = True

    for name, subcommand in subcommand_map.items():
        p = subparsers.add_parser(name, help=subcommand.brief_description)
        subcommand.populate_parser(p)

    return parser

def main(arguments):
    subcommand_lookup = generate_subcommands()
    parser = populate_parser(subcommand_lookup)
    args = parser.parse_args(arguments)
    return subcommand_lookup[args.subcommand].run(args)

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
