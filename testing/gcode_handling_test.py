import os
from curacc.gcode_handling import GCodeProjector
import numpy as np

model_path = os.path.join('gcode', 'CE3V3SE_willow_centered.gcode')
bed_path = os.path.join('bed_mesh', 'Curved_Print_Bed v13.stl')
curved_gcode_path = os.path.join('gcode', 'CE3V3SE_willow_centered_CURVED.gcode')

projector = GCodeProjector(gcode_file=model_path, bed_mesh_file=bed_path, layer_height=0.2,
                           maximum_gcode_point_distance=1, interpolate_max_count=12)

projector.run_all(output_file=curved_gcode_path)

