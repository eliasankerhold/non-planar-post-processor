from gcodeparser import GcodeParser, GcodeLine
import open3d as o3d
import numpy as np
import plotly_express as px
import matplotlib.pyplot as plt
import plotly.graph_objs as go
from stl.mesh import Mesh


class GCodeProjector:
    def __init__(self, gcode_file: str, bed_mesh_file: str, layer_height: float, maximum_gcode_point_distance: float,
                 z_homing_position: tuple[float, float], standard_z_homing: tuple[float, float],
                 overwrite: bool = False):
        self.gcode_path = gcode_file
        self.bed_mesh_path = bed_mesh_file
        self.overwrite = overwrite
        self.max_dist = maximum_gcode_point_distance
        self.raw_gcode = None
        self.gcode_parser = None
        self.n_layers = None
        self.original_points_list = []
        self.original_points_arr = None
        self.projected_points_arr = None
        self.layer_height = layer_height
        self.layer_z_vals = None
        self.points_per_layer = {}
        self.bed_mesh = None
        self.rays = None
        self.working_height = None
        self.z_homing_pos = np.array(z_homing_position)
        self.z_safety_dist = 4
        self.standard_z_homing = np.array(standard_z_homing)

        self.param_keys = {'X': 0, 'Y': 1, 'Z': 2, 'E': 4}

    def run_all(self, output_file: str, show_result=True):
        self.load_gcode()
        self.parse_points()
        self.read_points()
        self.sort_points_into_layers()
        self.project_points()
        self.update_gcode()
        self.export_projected_gcode(output_file)
        if show_result:
            self.plot_projected_points()

    def load_gcode(self):
        with open(self.gcode_path, 'r') as f:
            self.raw_gcode = f.read()

        self.gcode_parser = GcodeParser(self.raw_gcode, include_comments=True)
        print(f'Loaded and parsed gcode from {self.gcode_path}')

    def parse_points(self):
        pre_adjusted_lines = []
        new_lines = []
        raw_points = []
        added_points = 0
        original_len = len(self.gcode_parser.lines)
        temp_line_buffer = {'command': '', 'comment': ''}

        self.bed_mesh = o3d.io.read_triangle_mesh(self.bed_mesh_path)
        print(f'Loaded bed mesh: {self.bed_mesh_path}')
        self.working_height = np.asarray(self.bed_mesh.vertices)[:, 2].max() * 1.5
        pre_adjusted_lines.append(GcodeLine(command=('M', 211), params={'S': 0}, comment='disable software endstops'))
        raw_points.append(np.array([np.nan, np.nan, np.nan], dtype=float))

        for i, line in enumerate(self.gcode_parser.lines):
            temp_line_buffer['command'] = line.command
            temp_line_buffer['comment'] = line.comment
            if "LAYER_COUNT" in line.comment:
                self.n_layers = int(line.comment.split(':')[1])
                print(f'Model has {self.n_layers} layers.')

            if line.command == ('G', 28):
                pre_adjusted_lines.append(GcodeLine(command=('G', 0),
                                                    params={'Z': np.round(float(self.working_height * 2), 5)},
                                                    comment='raise to safe working height'))
                raw_points.append(np.array([pre_adjusted_lines[-1].get_param('X'),
                                            pre_adjusted_lines[-1].get_param('Y'),
                                            pre_adjusted_lines[-1].get_param('E')],
                                           dtype=float))
                pre_adjusted_lines.append(GcodeLine(command=('G', '28 X Y'), params={},
                                                    comment='home x and y axes'))
                raw_points.append(np.array([np.nan, np.nan, np.nan], dtype=float))
                pre_adjusted_lines.append(GcodeLine(command=('M',f'206 '
                                                                 f'X{float(self.standard_z_homing[0] - self.z_homing_pos[0])} '
                                                                 f'Y{float(self.standard_z_homing[1] -self.z_homing_pos[1])} '
                                                                 f'Z0'), params={},
                                                    comment='apply home offsets in x and y'))
                raw_points.append(np.array([np.nan, np.nan, np.nan], dtype=float))
                pre_adjusted_lines.append(GcodeLine(command=('M', 500), params={},
                                                    comment='save home offsets'))
                raw_points.append(np.array([np.nan, np.nan, np.nan], dtype=float))
                pre_adjusted_lines.append(GcodeLine(command=('G', '28 Z'), params={},
                                                    comment='home z axis'))
                raw_points.append(np.array([np.nan, np.nan, np.nan], dtype=float))
                pre_adjusted_lines.append(GcodeLine(command=('G', 0),
                                                    params={'Z': np.round(float(self.working_height) * 2, 5)},
                                                    comment='raise to safe working height'))
                raw_points.append(np.array([self.z_homing_pos[0],
                                            self.z_homing_pos[1],
                                            self.working_height], dtype=float))
                pre_adjusted_lines.append(GcodeLine(command=('M', '206 X0 Y0 Z0'), params={},
                                                    comment='reset home offsets in x and y'))
                raw_points.append(np.array([np.nan, np.nan, np.nan], dtype=float))
                pre_adjusted_lines.append(GcodeLine(command=('M', 500), params={},
                                                    comment='save home offsets'))
                raw_points.append(np.array([self.z_homing_pos[0], self.z_homing_pos[1], np.nan], dtype=float))
                print('Replaced G28 with custom homing procedure.')

            else:
                pre_adjusted_lines.append(line)
                raw_points.append(np.array([line.get_param('X'), line.get_param('Y'), line.get_param('E')],
                                           dtype=float))
        pre_adjusted_lines.append(GcodeLine(command=('M', 211), params={'S': 1}, comment='enable software endstops'))
        raw_points.append(np.array([np.nan, np.nan, np.nan], dtype=float))
        pre_adjusted_lines.append(GcodeLine(command=('G', 0), params={}, comment='buffer line'))
        raw_points.append(np.array([np.nan, np.nan, np.nan], dtype=float))
        e_position = 0
        xy_position = raw_points[0][:2]
        x_dist, y_dist = 0, 0
        for i, line in enumerate(pre_adjusted_lines[:-1]):
            if not np.isnan(raw_points[i][0]):
                xy_position[0] = raw_points[i][0]
            if not np.isnan(raw_points[i][1]):
                xy_position[1] = raw_points[i][1]
            if not np.isnan(raw_points[i][2]):
                e_position = raw_points[i][2]
            new_lines.append(line)
            if not np.isnan(raw_points[i + 1][0]):
                x_dist = raw_points[i + 1][0] - xy_position[0]
            if not np.isnan(raw_points[i + 1][1]):
                y_dist = raw_points[i + 1][1] - xy_position[1]
            vec = np.array([x_dist, y_dist])
            xy_dist = np.linalg.norm(vec)
            if not np.isnan(raw_points[i + 1][0]) and not np.isnan(raw_points[i + 1][1]) and xy_dist > self.max_dist:
                next_line = pre_adjusted_lines[i + 1]
                nvec = vec / xy_dist
                n = int(np.ceil(xy_dist / self.max_dist))
                inter_dist = abs(xy_dist / n)
                if next_line.command == ('G', 1) and next_line.get_param('E') is not None:
                    e_dist = next_line.get_param('E') - e_position
                    inter_e_dist = e_dist / n
                # if xy_dist > self.z_safety_dist and next_line.get_param('E') is None:
                #     new_lines.append(GcodeLine(command=('G', 0),
                #                                params={'Z': np.round(self.working_height, 0)},
                #                                comment='z safety move'))
                for k in range(1, n):
                    new_line = self.copy_gcode_line(next_line)
                    new_line.comment += '--INTERPOLATED'
                    new_line.update_param('X', float(xy_position[0] + nvec[0] * k * inter_dist))
                    new_line.update_param('Y', float(xy_position[1] + nvec[1] * k * inter_dist))
                    if next_line.command == ('G', 1) and next_line.get_param('E'):
                        new_line.update_param('E', float(e_position + inter_e_dist))
                        e_position = new_line.get_param('E')
                    new_lines.append(new_line)
                    added_points += 1

        self.gcode_parser.lines = new_lines
        print(f'Original GCode has {original_len} points. Added {added_points} interpolated points.')

    def read_points(self):
        z_height = 0
        for i, line in enumerate(self.gcode_parser.lines):
            if line.get_param('Z') is None:
                z = z_height
            else:
                z = line.get_param('Z')
                z_height = z
            self.original_points_list.append(
                np.array([line.get_param('X'), line.get_param('Y'), z, i, line.get_param('E')], dtype=float))

        self.original_points_arr = np.array(self.original_points_list)

    def sort_points_into_layers(self):
        self.layer_z_vals = np.unique(self.original_points_arr[:, 2])
        for z in self.layer_z_vals:
            self.points_per_layer[z] = self.original_points_arr[np.where(self.original_points_arr[:, 2] == z)]

    def update_gcode(self):
        for point in self.projected_points_arr:
            ind = int(point[3])
            old_line = self.gcode_parser.lines[ind]
            new_params = old_line.params
            for key, i in self.param_keys.items():
                if np.isfinite(point[i]):
                    new_params[key] = float(np.round(point[i], 4))

            new_line = GcodeLine(command=old_line.command, params=new_params, comment=old_line.comment)
            self.gcode_parser.lines[ind] = new_line

    def export_projected_gcode(self, export_path: str):
        print('Writing gcode...')
        with open(export_path, 'w') as f:
            f.writelines([line.gcode_str + '\n' for line in self.gcode_parser.lines])

        print(f'Updated gcode written to {export_path}')

    def plot_original_points(self):
        fig3d = px.scatter_3d(x=self.original_points_arr[:, 0], y=self.original_points_arr[:, 1],
                              z=self.original_points_arr[:, 2])
        fig3d.update_traces(marker=dict(size=1))
        fig3d.show()

    def plot_projected_points(self):
        bed = Mesh.from_file(self.bed_mesh_path)
        vertices, i, j, k = self.stl2mesh3d(bed)
        x, y, z = vertices.T
        colorscale = [[0, '#e5dee5'], [1, '#e5dee5']]

        mesh = go.Mesh3d(x=x, y=y, z=z, i=i, j=j, k=k,
                         flatshading=True,
                         colorscale=colorscale,
                         intensity=z,
                         name='AT&T',
                         showscale=False)
        scatter = go.Scatter3d(x=self.projected_points_arr[:, 0],
                               y=self.projected_points_arr[:, 1],
                               z=self.projected_points_arr[:, 2])

        fig = go.Figure(data=[mesh, scatter])
        fig.update_yaxes(
            scaleanchor="x",
            scaleratio=1,
        )
        fig.data[0].update(lighting=dict(ambient=0.18,
                                         diffuse=1,
                                         fresnel=.1,
                                         specular=1,
                                         roughness=.1,
                                         facenormalsepsilon=0))
        fig.data[0].update(lightposition=dict(x=3000,
                                              y=3000,
                                              z=10000))

        fig.show()

    def plot_z_heights(self):
        fig, ax = plt.subplots()
        ax.scatter(np.arange(len(self.original_points_arr)), self.original_points_arr[:, 2])
        plt.show()

    def plot_top_view(self):
        fig, ax = plt.subplots()
        ax.scatter(self.original_points_arr[:, 0], self.original_points_arr[:, 1], s=10)
        ax.set_aspect('equal', adjustable='datalim')
        plt.show()

    def project_points(self):

        for z_height, layer_points in self.points_per_layer.items():
            print(f'Projecting layer z={z_height:010} ...')
            layer_points[:, 2] = self.working_height * 2
            rays = self._generate_ray_tensors(layer_points[:, :-2])
            scene = o3d.t.geometry.RaycastingScene()
            scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh_legacy=self.bed_mesh))
            normal_cast = scene.cast_rays(rays, nthreads=0)
            dists = normal_cast['t_hit'].numpy()
            dists[np.isnan(dists)] = 0
            self.points_per_layer[z_height][:, 2] += z_height - dists

        self.projected_points_arr = np.concatenate(tuple([np.array(k) for k in self.points_per_layer.values()]))
        print('\nProjecting completed!')

    @staticmethod
    def _generate_ray_tensors(points):
        normals = np.zeros_like(points)
        normals[:, 2] = -1

        rays = o3d.core.Tensor(np.concatenate((points, normals), axis=1),
                               dtype=o3d.core.Dtype.Float32)

        return rays

    @staticmethod
    def stl2mesh3d(stl_mesh):
        # stl_mesh is read by nympy-stl from a stl file; it is  an array of faces/triangles (i.e. three 3d points)
        # this function extracts the unique vertices and the lists I, J, K to define a Plotly mesh3d
        p, q, r = stl_mesh.vectors.shape  # (p, 3, 3)
        # the array stl_mesh.vectors.reshape(p*q, r) can contain multiple copies of the same vertex;
        # extract unique vertices from all mesh triangles
        vertices, ixr = np.unique(stl_mesh.vectors.reshape(p * q, r), return_inverse=True, axis=0)
        I = np.take(ixr, [3 * k for k in range(p)])
        J = np.take(ixr, [3 * k + 1 for k in range(p)])
        K = np.take(ixr, [3 * k + 2 for k in range(p)])
        return vertices, I, J, K

    @staticmethod
    def copy_gcode_line(line: GcodeLine):
        return GcodeLine(command=line.command, params=line.params.copy(), comment=line.comment)
