import copy
from typing import List
import os
import glob

import numpy as np
from numba import njit
import matplotlib.pyplot as plt
import open3d as o3d

from config import config


def sigmoid(x, alpha: float = 10):
    return 1 / (1 + np.exp(-alpha * x))


def read_pcd(lidar_file: str) -> o3d.geometry.PointCloud:
    lidar_data = np.fromfile(lidar_file, dtype=np.float32).reshape(-1, 4)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidar_data[:, :3])
    pcd.colors = o3d.utility.Vector3dVector(np.tile(lidar_data[:, 3], (3, 1)).T)

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
            np.tile(colors, (data.shape[0], 1)))
    else:
        raise RuntimeError(f"invalid shape: {colors.shape}")

    return pcd


def get_scan_id(vertical_deg: float) -> int:
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
    scan_ids = np.array([get_scan_id(deg) for deg in vertical_deg])
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


def get_features(pcd: o3d.geometry.PointCloud, draw: bool = False):
    # 原点近傍の点は除く
    non_close_idx = np.array([idx for idx, point in enumerate(np.array(pcd.points))
                              if np.linalg.norm(point) > config["minimum_range"]])
    scans = pcd2laser(pcd.select_by_index(non_close_idx))
    features = [get_features_per_scan(scan[:, :3], draw=False)
                for scan in scans]

    sharp_edges = np.vstack([feature[0] for feature in features])
    less_sharp_edges = np.vstack([feature[1] for feature in features])
    planes = np.vstack([feature[2] for feature in features])

    if draw:
        edge_pcd = ndarray2pcd(sharp_edges, np.tile((1, 0, 0), (sharp_edges.shape[0], 1)))
        less_edge_pcd = ndarray2pcd(less_sharp_edges, np.tile((0, 1, 0), (less_sharp_edges.shape[0], 1)))
        plane_pcd = ndarray2pcd(planes, np.tile((0, 0, 1), (planes.shape[0], 1)))

        o3d.visualization.draw_geometries([pcd, plane_pcd, less_edge_pcd, edge_pcd])

    return sharp_edges, less_sharp_edges, planes


def get_curvature(scan: np.ndarray, draw: bool = False) -> np.ndarray:
    """
    中心点を x0 とすると Σ_{i-5}^{i+5}(xi-x0)を計算.
    近似的な法線方向・曲率を計算

    :param scan:
    :param draw:
    :return:
    """
    diff = np.array([
        np.sum(np.roll(scan, shift=-(i - 5), axis=0)[0:11, :], axis=0) - 10 * scan[i, :]
        for i in range(scan.shape[0])])
    curvature = np.linalg.norm(diff, axis=1) ** 2

    # 可視化
    if draw:
        color = np.zeros_like(scan)
        color[:, 0] = curvature / np.max(curvature)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(scan)
        pcd.colors = o3d.utility.Vector3dVector(color)
        o3d.visualization.draw_geometries([pcd])

    return curvature


def get_features_per_scan(scan: np.ndarray, draw: bool = False) -> np.ndarray:
    data = copy.deepcopy(scan)
    curvature = get_curvature(scan)
    # 曲率を追加
    data = np.vstack([data.T, curvature]).T

    split_num = 6
    # 方位を分割
    scan_seg_list = np.array_split(data, split_num)
    # 曲率でソート
    sort_idx_list: List[np.ndarray] = [np.argsort(-scan_seg_list[i][:, 3])
                                       for i in range(split_num)]

    features = [get_edge(scan_seg, sort_idx)
                for scan_seg, sort_idx in zip(scan_seg_list, sort_idx_list)]
    sharp_edge = np.array(np.vstack([feature[0] for feature in features]))
    less_sharp_edge = np.array(np.vstack([feature[1] for feature in features]))
    planes = np.vstack([get_plane(scan_seg, sort_idx)
                        for scan_seg, sort_idx in zip(scan_seg_list, sort_idx_list)])

    if draw:
        pcd = ndarray2pcd(data[:, :3], np.zeros_like(data[:, :3]))
        edge_pcd = ndarray2pcd(sharp_edge, np.tile((1, 0, 0), (sharp_edge.shape[0], 1)))
        less_edge_pcd = ndarray2pcd(less_sharp_edge, np.tile((0, 1, 0), (less_sharp_edge.shape[0], 1)))
        plane_pcd = ndarray2pcd(planes, np.tile((0, 0, 1), (planes.shape[0], 1)))

        o3d.visualization.draw_geometries([pcd, plane_pcd, less_edge_pcd, edge_pcd])
    return sharp_edge, less_sharp_edge, planes


def get_edge(data: np.ndarray, idx_list: np.ndarray):
    corner_pts, less_corner_pts = [], []
    picked_num = 0
    checked = np.zeros_like(idx_list)
    label = np.zeros_like(idx_list)
    # 曲率の大きい順にチェック
    for idx in idx_list:
        # チェック済み or 曲率が小さい場合はスキップ
        if checked[idx] or data[idx, 3] <= 0.1:
            pass
        picked_num = picked_num + 1
        if picked_num <= 2:
            label[idx] = 2
            corner_pts.append(data[idx, :3])
            less_corner_pts.append(data[idx, :3])
        elif picked_num <= 20:
            label[idx] = 1
            less_corner_pts.append(data[idx, :3])
        else:
            # 多すぎたら終了
            break

        is_neighbor = np.array([nearby
                                for nearby in np.arange(idx - 5, idx + 5) % len(idx_list)
                                if np.linalg.norm(data[idx, :3] - data[nearby, :3]) ** 2 < 0.05])
        # 検出点の近くはチェック済みに追加
        checked[is_neighbor] = 1
    return np.array(corner_pts), np.array(less_corner_pts)


def get_plane(data: np.ndarray, idx_list: np.ndarray):
    plane_pts = []
    picked_num = 0
    checked = np.zeros_like(idx_list)
    label = np.zeros_like(idx_list)
    # 曲率の小さい順にチェック
    for idx in idx_list[::-1]:
        # チェック済み or 曲率が大きい場合はスキップ
        if checked[idx] or data[idx, 3] >= 0.1:
            pass
        picked_num = picked_num + 1
        label[idx] = -1
        plane_pts.append(data[idx, :3])

        if picked_num >= 4:
            # 多すぎたら終了
            break

        is_neighbor = np.array([nearby
                                for nearby in np.arange(idx - 5, idx + 5) % len(idx_list)
                                if np.linalg.norm(data[idx, :3] - data[nearby, :3]) ** 2 < 0.05])
        # 検出点の近くはチェック済みに追加
        checked[is_neighbor] = 1
    return np.array(plane_pts)


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
    get_features(pcd, draw=True)

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
