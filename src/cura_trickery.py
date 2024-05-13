import os
import open3d as o3d
import numpy as np
import copy
from numpy.typing import NDArray
import matplotlib.pyplot as plt
import plotly.express as px


class ModelHandler:
    def __init__(self, source_dir: str, export_dir: str, sampling_distance: float, bed_size: tuple,
                 file_ending: str = 'stl', origin: NDArray = np.array([0, 0]), layer_height: float = 0.2):
        self.source_dir = source_dir
        self.export_dir = export_dir
        self.loaded_files = {}
        self.file_ending = file_ending
        self.file_paths = []
        self.sampling_distance = sampling_distance
        self.bed_size = bed_size
        self.origin = origin
        self.layer_height = layer_height

        self.casting_points = None
        self.rays = None
        self.x_samples = None
        self.y_samples = None

        self.miss_inds = None
        self.hit_inds = None

        self.projected_points = None

        self._generate_casting_comb()
        self._generate_ray_tensors()

    def _get_file_paths(self):
        self.file_paths = [os.path.join(self.source_dir, p) for p in os.listdir(self.source_dir) if
                           p.endswith(self.file_ending)]

    def load_files(self):
        self._get_file_paths()
        for p in self.file_paths:
            self.loaded_files[os.path.basename(p)] = o3d.io.read_triangle_mesh(p)

    def _generate_casting_comb(self):
        n_sampling_lines = int(self.bed_size[1] / self.sampling_distance)
        n_sampling_points = int(np.ceil(self.bed_size[0] / self.sampling_distance))

        sampling_points = np.zeros((n_sampling_lines, n_sampling_points, 3))

        i, k = 0, 1

        sampling_points[:, :, i] = np.linspace(0, self.bed_size[i], n_sampling_points)
        for j in range(n_sampling_lines):
            sampling_points[j, :, k] = j * self.sampling_distance

        sampling_points[:, :, 2] = 0

        self.x_samples = sampling_points[0, :, i]
        self.y_samples = sampling_points[:, 0, k]

        self.casting_points = sampling_points

    def _generate_ray_tensors(self):
        normals = np.zeros_like(self.casting_points)
        normals[:, :, 2] = 1

        rays = o3d.core.Tensor(np.concatenate((self.casting_points, normals), axis=2),
                               dtype=o3d.core.Dtype.Float32)

        self.rays = rays

    def do_ray_casting(self, slice: o3d.geometry.TriangleMesh):
        scene = o3d.t.geometry.RaycastingScene()
        translate_vec = np.zeros(3)
        translate_vec[:2] = self.origin[:2] - slice.get_center()[:2]
        offset_mesh = copy.deepcopy(slice).translate(translate_vec)
        scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh_legacy=offset_mesh))
        normal_cast = scene.cast_rays(self.rays, nthreads=0)
        dists = normal_cast['t_hit'].numpy()

        sampling_points = np.zeros_like(self.casting_points)
        sampling_points[:, :, 2] += dists

        sampling_points[sampling_points[:, :, 2] == np.inf] = np.nan
        # miss_inds = np.where(np.isnan(sampling_points))
        hit_inds = np.where(~np.isnan(sampling_points[:, :, 2]))

        projected_points = self.casting_points[hit_inds]

        return projected_points

    def extrude_mesh(self, points: NDArray, axis: NDArray = np.array([0, 0, 1])):
        n_axis = axis / np.linalg.norm(axis)
        extruded_points = points + n_axis * self.layer_height
        mesh_points = np.concatenate((extruded_points, points))
        positive_normals = np.zeros_like(points)
        positive_normals[:, 2] = 1
        normals = np.concatenate((positive_normals, positive_normals * -1))
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(mesh_points)
        pcd.normals = o3d.utility.Vector3dVector(normals)
        # o3d.visualization.draw_geometries([pcd])
        radii = [self.sampling_distance * 0.9]
        rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd, o3d.utility.DoubleVector(radii))
        # rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, 0.5)
        # rec_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
        rec_mesh = rec_mesh.compute_vertex_normals()
        rec_mesh = rec_mesh.compute_triangle_normals()
        # rec_mesh.estimate_normals()
        o3d.visualization.draw_geometries([pcd, rec_mesh])

        o3d.io.write_triangle_mesh('./test/test.stl', rec_mesh)

        return mesh_points

    @staticmethod
    def show_ray_casting_result(projected_points: NDArray):
        fig3d = px.scatter_3d(x=projected_points[:, 0], y=projected_points[:, 1], z=projected_points[:, 2])
        fig3d.update_traces(marker=dict(size=1))
        fig3d.show()


