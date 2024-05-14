import os
from nonplanarpp.gcode_handling import GCodeProjector
import numpy as np

model_path = os.path.join('gcode', 'CE3V3SE_Saucer_110mm.gcode')
bed_path = os.path.join('bed_mesh', 'Curved_Print_Bed v13.stl')
curved_gcode_path = os.path.join('gcode', 'CE3V3SE_Saucer_110mm_CURVED.gcode')

projector = GCodeProjector(gcode_file=model_path, bed_mesh_file=bed_path, layer_height=0.2,
                           maximum_gcode_point_distance=1)

projector.run_all(output_file=curved_gcode_path, show_result=True)
