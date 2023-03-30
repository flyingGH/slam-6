"""
点群変換処理モジュール
"""
from typing import List

import numpy as np
import open3d as o3d

from .config import config


def read_pcd(lidar_file: str) -> o3d.geometry.PointCloud:
    lidar_data = np.fromfile(lidar_file, dtype=np.float32).reshape(-1, 4)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidar_data[:, :3])
    pcd.colors = o3d.utility.Vector3dVector(
        np.tile(lidar_data[:, 3], (3, 1)).T)

    return pcd


def ndarray2pcd(points: np.ndarray, colors: np.ndarray) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # 全点の色を指定する場合
    if np.all(points.shape == colors.shape):
        pcd.colors = o3d.utility.Vector3dVector(colors)
    # すべての点を同一色で着色する場合
    elif colors.dims == 1:
        pcd.colors = o3d.utility.Vector3dVector(
            np.tile(colors, (points.shape[0], 1)))
    else:
        raise RuntimeError(f"invalid shape: {colors.shape}")

    return pcd


def _get_scan_id(vertical_deg: float) -> int:
    """
    cf. https://github.com/HKUST-Aerial-Robotics/A-LOAM/blob/devel/src/scanRegistration.cpp

    :param vertical_deg:
    :return:
    """
    if config["n_scans"] == 64:
        if vertical_deg >= -8.83:
            return int((2 - vertical_deg) * 3 + 0.5)
        else:
            return int(config["n_scans"] / 2) + int((-8.83 - vertical_deg) * 2 + 0.5)
    else:
        raise RuntimeError(f"invalid n_scans: {config['n_scans']}")


def pcd2laser(pcd: o3d.geometry.PointCloud) -> List[np.ndarray]:
    data = np.array(pcd.points)

    # 仰俯角計算
    r_xy = np.linalg.norm(data[:, :2], axis=1)
    vertical_deg = np.rad2deg(np.arctan(data[:, 2] / r_xy))
    # レーザID計算
    scan_ids = np.array([_get_scan_id(deg) for deg in vertical_deg])
    # レーザ ID 毎に点群を分割
    scans = [data[scan_ids == scan_id, :]
             for scan_id in range(config["n_scans"])]

    # 可視化
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(scan)
    # horizon_deg = np.rad2deg(-np.arctan2(data[:, 1], data[:, 0]))
    # pcd.colors = o3d.utility.Vector3dVector(np.tile(horizon_deg[scan_ids == draw_id] / 360 + 0.5, (3, 1)).T)
    # o3d.visualization.draw_geometries([pcd])

    return scans
