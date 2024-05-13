# from cura_trickery import ModelHandler

# import numpy as np

# handler = ModelHandler(source_dir='./', export_dir='./', sampling_distance=0.1, bed_size=(200, 200),
#                        origin=np.array([100, 100]))

# handler.load_files()
# print(handler.loaded_files)
# points = handler.do_ray_casting(list(handler.loaded_files.values())[0])
# mesh_points = handler.extrude_mesh(points=points)
# # handler.show_ray_casting_result(projected_points=mesh_points)

from curacc.settings_parser import log_to_json, find_missing_settings
import os

log_export_file = os.path.join('../cura_resources', 'all_settings')
output_file = 'settings_output.json'
extra_settings = 'extra_settings.json'

log_to_json(log_file=log_export_file, output_path=output_file)
find_missing_settings(output_path=extra_settings, settings_path=output_file, test_model_path='Body1.stl')