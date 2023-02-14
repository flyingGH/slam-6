import operator
import os
import cv2
import open3d as o3d
import pandas as pd
import numpy as np

import stereo
import tracking


def read_calib_matrices(calib_path):
    calib_df = pd.read_csv(calib_path, delim_whitespace=True, header=None)
    calib_matrices = [row[1:].values.reshape((3, 4)) for _, row in calib_df.iterrows()]
    return calib_matrices


def demo_stereo_matching():
    root_path = "/mnt/d/datasets/KITTI/odometry"
    left_image_path = os.path.join(root_path, "sequences/00/image_0/000000.png")
    right_image_path = os.path.join(root_path, "sequences/00/image_1/000000.png")

    calib_path = os.path.join(root_path, "sequences/00/calib.txt")
    calib_matrices = read_calib_matrices(calib_path)

    left_image = cv2.imread(left_image_path, cv2.IMREAD_GRAYSCALE)
    right_image = cv2.imread(right_image_path, cv2.IMREAD_GRAYSCALE)

    left_key_points, right_key_points = tracking.feature_tracking(
        left_image, right_image, key_points_1=None, draw=True)
    left_key_points, right_key_points = tracking.direct_tracking(
        left_image, right_image, key_points_1=None, draw=True)

    left_key_points, right_key_points, landmarks = stereo.triangulation(
        left_key_points, right_key_points, calib_matrices[1])

    stereo_estimator = stereo.StereoEstimator()
    points = stereo_estimator.images2points(left_image, right_image, calib_matrices[1], draw=False)

    landmark_pcd = o3d.geometry.PointCloud()
    landmark_pcd.points = o3d.utility.Vector3dVector(landmarks)
    landmark_pcd.paint_uniform_color((1, 0, 0))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])
    pcd.colors = o3d.utility.Vector3dVector(np.tile(points[:, 3], (3, 1)).T)
    
    o3d.visualization.draw_geometries([pcd, landmark_pcd])


if __name__ == "__main__":
    demo_stereo_matching()
