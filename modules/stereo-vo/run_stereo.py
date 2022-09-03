import operator
import os
import cv2
import pandas as pd
import numpy as np
import pangolin
import OpenGL.GL as gl
import open3d as o3d


def main():
    root_path = "/mnt/d/datasets/KITTI/odometry"
    left_image_path = os.path.join(root_path, "sequences/00/image_0/000000.png")
    right_image_path = os.path.join(root_path, "sequences/00/image_1/000000.png")

    calib_path = os.path.join(root_path, "sequences/00/calib.txt")
    calib_matrices = read_calib_matrices(calib_path)

    left_image = cv2.imread(left_image_path, cv2.IMREAD_GRAYSCALE)
    right_image = cv2.imread(right_image_path, cv2.IMREAD_GRAYSCALE)

    matcher = ImageMatcher()
    left_key_points, right_key_points, matches = matcher.get_matches(left_image, right_image, draw=True)

    stereo_estimator = StereoEstimator()
    pcd = stereo_estimator.get_point_cloud(left_image, right_image, calib_matrices[1], draw=True)


class ImageMatcher:
    def __init__(self):
        self.orb = cv2.ORB_create()
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    def get_matches(self, image_1, image_2, draw=False):
        key_points_1, descriptors_1 = self.orb.detectAndCompute(image_1, None)
        key_points_2, descriptors_2 = self.orb.detectAndCompute(image_2, None)
        # left_image_w_key_points = cv2.drawKeypoints(image_1, key_points_1,
        #                                             outImage=None, color=(0, 255, 0), flags=0)

        matches = list(self.matcher.match(descriptors_1, descriptors_2))

        # remove outliers
        matches.sort(key=operator.attrgetter("distance"))
        min_distance = matches[0].distance
        # min_distance = min(matches, key=operator.attrgetter("distance")).distance
        matches = list(filter(lambda x: x.distance <= max(2 * min_distance, 30), matches))

        if draw:
            match_image = cv2.drawMatches(image_1, key_points_1, image_2, key_points_2,
                                          matches, outImg=None, flags=2)
            cv2.imshow("result of feature matching", match_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return key_points_1, key_points_2, matches


def read_calib_matrices(calib_path):
    calib_df = pd.read_csv(calib_path, delim_whitespace=True, header=None)
    calib_matrices = [row[1:].values.reshape((3, 4)) for _, row in calib_df.iterrows()]
    return calib_matrices


def get_point(u, v, disparity, calib_mat, color):
    x = (u - calib_mat[0, 2]) / calib_mat[0, 0]
    y = (v - calib_mat[1, 2]) / calib_mat[1, 1]
    depth = -calib_mat[0, 3] / disparity
    return x * depth, depth, -y * depth, color / 255


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
            get_point(col, row, disparity_image[row, col], calib_matrix, left_image[row, col])
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


if __name__ == "__main__":
    main()
