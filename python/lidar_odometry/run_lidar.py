"""
LiDAR SLAM 実行モジュール
"""
import os
import glob

import numpy as np
import open3d as o3d

from .config import config

from . import features, pcd_io


def sigmoid(x, alpha: float = 10):
    return 1 / (1 + np.exp(-alpha * x))


def update_pcd(lidar_file: str, pcd: o3d.geometry.PointCloud) -> None:
    lidar_data = np.fromfile(lidar_file, dtype=np.float32).reshape(-1, 4)

    pcd.points = o3d.utility.Vector3dVector(lidar_data[:, :3])
    pcd.colors = o3d.utility.Vector3dVector(
        np.tile(lidar_data[:, 3], (3, 1)).T)


def main():
    lidar_files = glob.glob(os.path.join(
        config["root_path"], config["sequence_num"], "velodyne", "*.bin"))

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    pcd = pcd_io.read_pcd(lidar_files[0])
    sharp_edges, less_sharp_edges, planes = features.get_features(
        pcd, draw=False)
    edge_pcd = pcd_io.ndarray2pcd(sharp_edges, np.tile(
        (1, 0, 0), (sharp_edges.shape[0], 1)))
    less_edge_pcd = pcd_io.ndarray2pcd(less_sharp_edges, np.tile(
        (0, 1, 0), (less_sharp_edges.shape[0], 1)))
    plane_pcd = pcd_io.ndarray2pcd(
        planes, np.tile((0, 0, 1), (planes.shape[0], 1)))

    vis.add_geometry(pcd)
    vis.add_geometry(edge_pcd)
    vis.add_geometry(less_edge_pcd)
    vis.add_geometry(plane_pcd)

    frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
    vis.add_geometry(frame)

    for lidar_file in lidar_files[1:]:
        update_pcd(lidar_file, pcd)
        sharp_edges, less_sharp_edges, planes = features.get_features(
            pcd, draw=False)
        edge_pcd = pcd_io.ndarray2pcd(sharp_edges, np.tile(
            (1, 0, 0), (sharp_edges.shape[0], 1)))
        less_edge_pcd = pcd_io.ndarray2pcd(less_sharp_edges, np.tile(
            (0, 1, 0), (less_sharp_edges.shape[0], 1)))
        plane_pcd = pcd_io.ndarray2pcd(
            planes, np.tile((0, 0, 1), (planes.shape[0], 1)))

        vis.update_geometry(pcd)
        vis.update_geometry(edge_pcd)
        vis.update_geometry(less_edge_pcd)
        vis.update_geometry(plane_pcd)

        vis.poll_events()
        vis.update_renderer()

    vis.destroy_window()


if __name__ == "__main__":
    main()
