import numpy as np
import cv2
import open3d as o3d


def pixel2point(u, v, disparity, calib_mat):
    x = (u - calib_mat[0, 2]) / calib_mat[0, 0]
    y = (v - calib_mat[1, 2]) / calib_mat[1, 1]
    depth = -calib_mat[0, 3] / disparity
    # v is corresponding to height (switch y and depth)
    return x * depth, depth, -y * depth


def triangulation(left_key_points, right_key_points, calib_matrix):
    disparity = np.array([left_pt.pt[0] - right_pt.pt[0]
                          for left_pt, right_pt in zip(left_key_points, right_key_points)])
    landmarks = np.array([pixel2point(*left_pt.pt, left_pt.pt[0] - right_pt.pt[0], calib_matrix)
                          for left_pt, right_pt in zip(left_key_points, right_key_points)])

    is_valid = np.where((10 <= disparity) & (disparity <= 96), True, False)
    left_key_points = [left_key_points[idx] for idx, valid in enumerate(is_valid) if valid]
    right_key_points = [right_key_points[idx] for idx, valid in enumerate(is_valid) if valid]
    return left_key_points, right_key_points, landmarks[is_valid]


class StereoEstimator:
    def __init__(self):
        self.disparity_estimator = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=96, blockSize=9,
            P1=8 * 9 * 9, P2=32 * 9 * 9,
            disp12MaxDiff=1, preFilterCap=63, uniquenessRatio=10,
            speckleWindowSize=100, speckleRange=32)

    def get_disparity(self, left_image, right_image):
        disparity_image = self.disparity_estimator.compute(left_image, right_image)
        # cf. https://dev.classmethod.jp/articles/stereo-depth-estimation-with-opencv-2nd/
        disparity_image = disparity_image / 16
        return disparity_image

    def get_point_cloud(self, left_image, right_image, calib_matrix, draw=False):
        disparity_image = self.get_disparity(left_image, right_image)
        points = np.array([
            (*pixel2point(col, row, disparity_image[row, col], calib_matrix), left_image[row, col] / 255)
            for row in range(0, disparity_image.shape[0])
            for col in range(0, disparity_image.shape[1])
            if 10 <= disparity_image[row, col] <= 96
        ])

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        pcd.colors = o3d.utility.Vector3dVector(np.tile(points[:, 3], (3, 1)).T)
        if draw:
            o3d.visualization.draw_geometries([pcd])
        return pcd
