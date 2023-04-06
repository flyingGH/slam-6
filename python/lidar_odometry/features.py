"""
特徴点(エッジ・平面)計算モジュール
"""
import copy
from typing import List, Tuple

import numpy as np
from numba import njit
import open3d as o3d

from .config import config
from . import pcd_io


def _get_curvature(scan: np.ndarray, draw: bool = False) -> np.ndarray:
    """
    中心点を x0 とすると Σ_{i-5}^{i+5}(xi-x0)を計算.
    近似的な法線方向・曲率を計算

    :param scan:
    :param draw:
    :return:
    """
    diff = np.array([
        np.sum(np.roll(scan, shift=-(i - 5), axis=0)
               [0:11, :], axis=0) - 10 * scan[i, :]
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


@njit(cache=True, fastmath=True)
def _get_edge(data: np.ndarray, idx_list: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    corner_pts = np.empty((0, 3), dtype=np.float64)
    less_corner_pts = np.empty((0, 3), dtype=np.float64)
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
            corner_pts = np.vstack([corner_pts, data[idx, :3]])
            less_corner_pts = np.vstack([less_corner_pts, data[idx, :3]])
        elif picked_num <= 20:
            label[idx] = 1
            less_corner_pts = np.vstack([less_corner_pts, data[idx, :3]])
    #     else:
    #         # 多すぎたら終了
    #         break
    #
    #     for nearby in np.arange(idx - 5, idx + 5) % len(idx_list):
    #         if np.linalg.norm(data[idx, :3] - data[nearby, :3]) ** 2 < 0.05:
    #             # 検出点の近くはチェック済みに追加
    #             checked[nearby] = 1
    return corner_pts, less_corner_pts


# @njit(cache=True, fastmath=True)
def _get_plane(data: np.ndarray, idx_list: np.ndarray) -> np.ndarray:
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

        for nearby in np.arange(idx - 5, idx + 5) % len(idx_list):
            if np.linalg.norm(data[idx, :3] - data[nearby, :3]) ** 2 < 0.05:
                # 検出点の近くはチェック済みに追加
                checked[nearby] = 1
    return np.array(plane_pts)


def _get_features_per_scan(scan: np.ndarray, draw: bool = False) -> np.ndarray:
    data = copy.deepcopy(scan)
    curvature = _get_curvature(scan)
    # 曲率を追加
    data = np.vstack([data.T, curvature]).T

    split_num = 6
    # 方位を分割
    scan_seg_list = np.array_split(data, split_num)
    # 曲率でソート
    sort_idx_list: List[np.ndarray] = [np.argsort(-scan_seg_list[i][:, 3])
                                       for i in range(split_num)]

    features = [_get_edge(scan_seg, sort_idx)
                for scan_seg, sort_idx in zip(scan_seg_list, sort_idx_list)]
    sharp_edge = np.array(np.vstack([feature[0] for feature in features]))
    less_sharp_edge = np.array(np.vstack([feature[1] for feature in features]))
    planes = np.vstack([_get_plane(scan_seg, sort_idx)
                        for scan_seg, sort_idx in zip(scan_seg_list, sort_idx_list)])

    if draw:
        pcd = pcd_io.ndarray2pcd(data[:, :3], np.zeros_like(data[:, :3]))
        edge_pcd = pcd_io.ndarray2pcd(sharp_edge, np.tile(
            (1, 0, 0), (sharp_edge.shape[0], 1)))
        less_edge_pcd = pcd_io.ndarray2pcd(less_sharp_edge, np.tile(
            (0, 1, 0), (less_sharp_edge.shape[0], 1)))
        plane_pcd = pcd_io.ndarray2pcd(
            planes, np.tile((0, 0, 1), (planes.shape[0], 1)))

        o3d.visualization.draw_geometries(
            [pcd, plane_pcd, less_edge_pcd, edge_pcd])
    return sharp_edge, less_sharp_edge, planes


def get_features(pcd: o3d.geometry.PointCloud, draw: bool = False):
    # 原点近傍の点は除く
    non_close_idx = np.array([idx for idx, point in enumerate(np.array(pcd.points))
                              if np.linalg.norm(point) > config["minimum_range"]])
    scans = pcd_io.pcd2laser(pcd.select_by_index(non_close_idx))
    features = [_get_features_per_scan(scan[:, :3], draw=False)
                for scan in scans]

    sharp_edges = np.vstack([feature[0] for feature in features])
    less_sharp_edges = np.vstack([feature[1] for feature in features])
    planes = np.vstack([feature[2] for feature in features])

    if draw:
        edge_pcd = pcd_io.ndarray2pcd(sharp_edges, np.tile(
            (1, 0, 0), (sharp_edges.shape[0], 1)))
        less_edge_pcd = pcd_io.ndarray2pcd(less_sharp_edges, np.tile(
            (0, 1, 0), (less_sharp_edges.shape[0], 1)))
        plane_pcd = pcd_io.ndarray2pcd(
            planes, np.tile((0, 0, 1), (planes.shape[0], 1)))

        o3d.visualization.draw_geometries(
            [pcd, plane_pcd, less_edge_pcd, edge_pcd])

    return sharp_edges, less_sharp_edges, planes
