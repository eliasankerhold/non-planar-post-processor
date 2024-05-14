from argparse import ArgumentParser
from nonplanarpp.gcode_handling import GCodeProjector
import os
import sys

parser = ArgumentParser(prog="Non Planar Post Processor",
                        description="Projects gcode onto a non-planar print bed. To load arguments from file, use @ "
                                    "prefix.",
                        epilog="Be careful when printing, this approach is highly experimental and can"
                               "cause crashes, issues with bed leveling or other unexpected behavior.",
                        fromfile_prefix_chars="@")

file_io = parser.add_argument_group('File handling')
file_io.add_argument('-g', '--gcode', help='File path of GCode to be processed.', required=True)
file_io.add_argument('-b', '--bed', help='STL file of the non-planar bed to be printed on.', required=True)
file_io.add_argument('--output', help='Output path for processed gcode.', required=False)
file_io.add_argument('--overwrite_source', help='Determines whether source gcode is overwritten. Defaults to false.',
                     required=False, default=0, type=int, choices=[0, 1])

pp_settings = parser.add_argument_group('Post processing settings')
pp_settings.add_argument('-l', '--layer_height', help='Layer height in mm.', required=True, type=float)
pp_settings.add_argument('-r', '--resolution', help='Interpolation resolution in mm.', required=True, type=float)
pp_settings.add_argument('--plot_result', help='Toggles visual output of non-planar gcode. Defaults to false.',
                         required=False, default=0, type=int, choices=[0, 1])

args = parser.parse_args()


def check_file(path, file_type):
    if not os.path.isfile(path):
        sys.exit(f'Fatal error: {path} is not a valid file.')

    if not path.endswith(file_type):
        sys.exit(f'Fatal error: {path} is not a {file_type} file.')


check_file(args.gcode, '.gcode')
check_file(args.bed, '.stl')

if args.output is None:
    output_file = os.path.join(os.path.dirname(args.gcode),
                               os.path.splitext(os.path.basename(args.gcode))[0] + '_POST-PROCESSED.gcode')

else:
    output_file = args.output

if bool(args.overwrite_source):
    output_file = args.gcode

projector = GCodeProjector(gcode_file=args.gcode,
                           bed_mesh_file=args.bed,
                           layer_height=args.layer_height,
                           maximum_gcode_point_distance=args.resolution)

projector.run_all(output_file=str(output_file), show_result=bool(args.plot_result))

print(
    "\nWARNING!\n"
    "Be careful when printing, this approach is highly experimental and can cause crashes, issues with bed leveling "
    "and other unexpected behavior. Check the result in a gcode previewer before printing.")
