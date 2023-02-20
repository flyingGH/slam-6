import os
import glob

import numpy as np
import open3d as o3d

from config import config


def read_pcd(lidar_file: str) -> o3d.geometry.PointCloud:
    lidar_data = np.fromfile(lidar_file, dtype=np.float32).reshape(-1, 4)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidar_data[:, :3])
    pcd.colors = o3d.utility.Vector3dVector(np.tile(lidar_data[:, 3], (3, 1)).T)
    return pcd


def update_pcd(lidar_file: str, pcd: o3d.geometry.PointCloud) -> None:
    lidar_data = np.fromfile(lidar_file, dtype=np.float32).reshape(-1, 4)

    pcd.points = o3d.utility.Vector3dVector(lidar_data[:, :3])
    pcd.colors = o3d.utility.Vector3dVector(np.tile(lidar_data[:, 3], (3, 1)).T)


def main():
    lidar_files = glob.glob(os.path.join(
        config["root_path"], config["sequence_num"], "velodyne", "*.bin"))

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    pcd = read_pcd(lidar_files[0])
    vis.add_geometry(pcd)

    frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
    vis.add_geometry(frame)

    for lidar_file in lidar_files[1:]:
        update_pcd(lidar_file, pcd)

        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

    vis.destroy_window()


if __name__ == "__main__":
    main()
