"""
三角測量モジュール
"""
import numpy as np
import cv2
import open3d as o3d

from .config import config


def pixel2point(u, v, disparity, calib_mat):
    """
    視差画像を3次元点へ変換

    :param u: ピクセルの列
    :param v: ピクセルの行
    :param disparity: 視差
    :param calib_mat: カメラ内部パラメータ
    :return: 3次元点
    """
    x = (u - calib_mat[0, 2]) / calib_mat[0, 0]
    y = (v - calib_mat[1, 2]) / calib_mat[1, 1]
    depth = -calib_mat[0, 3] / disparity
    # v is corresponding to height (switch y and depth)
    return x * depth, depth, -y * depth


def triangulation(left_key_points, right_key_points, calib_matrix):
    """
    左右画像からランドマーク点取得

    :param left_key_points: 左側画像の特徴点
    :param right_key_points: 右側画像の特徴点
    :param calib_matrix: カメラ内部パラメータ
    :return: フィルタ後の左側画像特徴点・右側画像特徴点・ランドマーク点
    """
    # 視差リスト
    disparity = np.array([left_pt[0] - right_pt[0]
                          for left_pt, right_pt in zip(left_key_points, right_key_points)])
    # ランドマーク点
    landmarks = np.array([pixel2point(*left_pt, left_pt[0] - right_pt[0], calib_matrix)
                          for left_pt, right_pt in zip(left_key_points, right_key_points)])

    # 視差の大きさでフィルタ
    is_valid = np.where((config["disparity_min"] <= disparity) & (
        disparity <= config["disparity_max"]), True, False)
    left_key_points = [left_key_points[idx]
                       for idx, valid in enumerate(is_valid) if valid]
    right_key_points = [right_key_points[idx]
                        for idx, valid in enumerate(is_valid) if valid]
    return left_key_points, right_key_points, landmarks[is_valid]


class StereoEstimator:
    def __init__(self):
        # cf. http://opencv.jp/opencv-2svn/cpp/camera_calibration_and_3d_reconstruction.html?highlight=stereobm
        self.disparity_estimator = cv2.StereoSGBM_create(
            **config["stereo_sgbm"])

    def get_disparity(self, left_image, right_image):
        """
        視差画像を計算

        :param left_image: 左側画像
        :param right_image: 右側画像
        :return: 視差画像
        """
        disparity_image = self.disparity_estimator.compute(
            left_image, right_image)
        # cf. https://dev.classmethod.jp/articles/stereo-depth-estimation-with-opencv-2nd/
        disparity_image = disparity_image / 16
        return disparity_image

    def images2points(self, left_image, right_image, calib_matrix, draw=False):
        """
        左右画像から特徴点による点群を推定

        :param left_image: 左側画像
        :param right_image: 右側画像
        :param calib_matrix: カメラ内部パラメータ
        :param draw: 描画の有無
        :return: (x,y,z,i)点群
        """
        # 視差画像
        disparity_image = self.get_disparity(left_image, right_image)
        points = np.array([
            (*pixel2point(col, row, disparity_image[row, col], calib_matrix),
             left_image[row, col] / 255)
            for row in range(0, disparity_image.shape[0])
            for col in range(0, disparity_image.shape[1])
            if 10 <= disparity_image[row, col] <= 96
        ])

        if draw:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points[:, :3])
            pcd.colors = o3d.utility.Vector3dVector(
                np.tile(points[:, 3], (3, 1)).T)
            o3d.visualization.draw_geometries([pcd])
        return points
