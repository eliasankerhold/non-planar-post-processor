from cura_trickery import ModelHandler

import numpy as np

handler = ModelHandler(source_dir='./', export_dir='./', sampling_distance=0.1, bed_size=(200, 200),
                       origin=np.array([100, 100]))

handler.load_files()
print(handler.loaded_files)
points = handler.do_ray_casting(list(handler.loaded_files.values())[0])
mesh_points = handler.extrude_mesh(points=points)
# handler.show_ray_casting_result(projected_points=mesh_points)
