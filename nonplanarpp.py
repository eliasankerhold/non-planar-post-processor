from argparse import ArgumentParser

import numpy as np
from configparser import ConfigParser

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
file_io.add_argument('-c', '--config', help='Config file for printer storing homing and priming areas.', required=True)
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


def parse_config_file(path):
    homing = {'safe_z_homing_x': None, 'safe_z_homing_y': None,
              'standard_home_x': None, 'standard_home_y': None}
    priming = {'safe_priming_corner_1_x': None, 'safe_priming_corner_1_y': None,
               'safe_priming_corner_2_x': None, 'safe_priming_corner_2_y': None}
    full_conf = {'HOMING': homing, 'PRIMING': priming}
    config = ConfigParser()
    try:
        config.read(path)
    except Exception as ex:
        print(ex)
        sys.exit('Fatal error: Cannot read config file.')

    for section in full_conf:
        for key, val in config[section].items():
            try:
                full_conf[section][key] = float(val)
            except ValueError as ex:
                print(ex)
                sys.exit(f"Fatal error: Cannot read {key}={val} from config file.")
            if val is None:
                sys.exit(f"Fatal error: Cannot read {key} from config file.")

    return full_conf


check_file(args.gcode, '.gcode')
check_file(args.bed, '.stl')
check_file(args.config, '.ini')
config = parse_config_file(args.config)

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
                           maximum_gcode_point_distance=args.resolution,
                           z_homing_position=(config['HOMING']['safe_z_homing_x'],
                                              config['HOMING']['safe_z_homing_y']),
                           standard_z_homing=(config['HOMING']['standard_home_x'],
                                              config['HOMING']['standard_home_y']))

projector.run_all(output_file=str(output_file), show_result=bool(args.plot_result))

print(
    "\nWARNING!\n"
    "Be careful when printing, this approach is highly experimental and can cause crashes, issues with bed leveling "
    "and other unexpected behavior. Check the result in a gcode previewer before printing.")
