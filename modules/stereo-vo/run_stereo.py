import operator
import os
import cv2
import open3d as o3d
import pandas as pd
import stereo_cv
import odometry


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

    matcher = odometry.ImageMatcher()
    left_key_points, right_key_points = matcher.get_matches(left_image, right_image, draw=False)
    left_key_points, right_key_points, landmarks = stereo_cv.triangulation(
        left_key_points, right_key_points, calib_matrices[1])

    stereo_estimator = stereo_cv.StereoEstimator()
    pcd = stereo_estimator.get_point_cloud(left_image, right_image, calib_matrices[1], draw=False)

    landmark_pcd = o3d.geometry.PointCloud()
    landmark_pcd.points = o3d.utility.Vector3dVector(landmarks)
    landmark_pcd.paint_uniform_color((1, 0, 0))
    o3d.visualization.draw_geometries([pcd, landmark_pcd])


if __name__ == "__main__":
    demo_stereo_matching()
