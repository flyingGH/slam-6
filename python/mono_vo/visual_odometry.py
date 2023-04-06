"""
| 単眼 Visual Odometry (Wheel Odometry 利用)
| from https://github.com/uoip/monoVO-python
"""
import numpy as np
import cv2
from typing import Tuple

from .config import config

# image processing mode
STAGE_FIRST_FRAME = 0
STAGE_SECOND_FRAME = 1
STAGE_DEFAULT_FRAME = 2


def feature_tracking(image_ref: np.ndarray, image_cur: np.ndarray, px_ref: np.ndarray)\
        -> Tuple[np.ndarray, np.ndarray]:
    """
    optical flow を計算

    :param image_ref: 直前の画像
    :param image_cur: 現在の画像
    :param px_ref: 直前の画像の特徴点
    :return: optical flow の始点・終点
    """
    # shape: [k,2] [k,1] [k,1]
    kp2, st, err = cv2.calcOpticalFlowPyrLK(
        image_ref, image_cur, prevPts=px_ref, nextPts=None, **config["lk_params"])

    # bool list for optical flow (same length with px_ref)
    st = st.reshape(st.shape[0])
    # extract feature points as start points of optical flows
    kp1 = px_ref[st == 1]
    # extract feature points as end points of optical flows
    kp2 = kp2[st == 1]

    return kp1, kp2


class PinholeCamera:
    """
    ピンホールカメラモデルのパラメータ
    """

    def __init__(self, width, height, fx, fy, cx, cy,
                 k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0):
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.distortion = (abs(k1) > 0.0000001)
        self.d = np.array([k1, k2, p1, p2, k3])

    def get_camera_mat(self):
        return np.array([[self.fx, 0, self.cx],
                         [0, self.fy, self.cy],
                         [0, 0, 1]])


class VisualOdometry:
    """
    visual odometry by sparce direct method (use feature points & optical flow)
    """

    def __init__(self, cam, annotations):
        """
        :param cam: inner camera model parameters
        :param annotations: path to annotation data for getting scale info
        """
        self.frame_stage = 0
        self.cam: PinholeCamera = cam
        self.new_frame = None
        self.last_frame = None
        self.cur_R = None
        self.cur_t = None
        self.px_ref = None
        self.px_cur = None
        self.focal = cam.fx
        self.pp = (cam.cx, cam.cy)
        self.trueX, self.trueY, self.trueZ = 0, 0, 0

        self.detector = cv2.FastFeatureDetector_create(
            threshold=25, nonmaxSuppression=True)

        with open(annotations) as f:
            self.annotations = f.readlines()

    def get_absolute_scale(self, frame_id):
        """
        get scale data (specialized for KITTI odometry dataset)

        :param frame_id:
        :return:
        """
        ss = self.annotations[frame_id - 1].strip().split()
        x_prev = float(ss[3])
        y_prev = float(ss[7])
        z_prev = float(ss[11])

        ss = self.annotations[frame_id].strip().split()
        x = float(ss[3])
        y = float(ss[7])
        z = float(ss[11])

        self.trueX, self.trueY, self.trueZ = x, y, z

        # 3D Pythagorean theorem
        return np.sqrt((x - x_prev) * (x - x_prev) + (y - y_prev) * (y - y_prev) + (z - z_prev) * (z - z_prev))

    def process_first_frame(self):
        """
        only detect feature points
        """
        self.px_ref: Tuple[cv2.KeyPoint] = self.detector.detect(self.new_frame)
        self.px_ref = np.array([x.pt for x in self.px_ref], dtype=np.float32)
        # move to the next processing stage
        self.frame_stage = STAGE_SECOND_FRAME

    def process_second_frame(self):
        """
        get optical flow and apply 5-point algorithm
        """
        # get optical flow
        self.px_ref, self.px_cur = feature_tracking(
            self.last_frame, self.new_frame, self.px_ref)

        # e_mat, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp, method=cv2.RANSAC,
        #                                    prob=0.999, threshold=1.0)
        e_mat, mask = cv2.findEssentialMat(
            self.px_cur, self.px_ref, cameraMatrix=self.cam.get_camera_mat(), method=cv2.RANSAC,
            prob=0.999, threshold=1.0)

        # 5-point algorithm
        _, self.cur_R, self.cur_t, mask = cv2.recoverPose(
            e_mat, self.px_cur, self.px_ref, focal=self.focal, pp=self.pp)

        self.px_ref = self.px_cur
        # move to the next processing stage
        self.frame_stage = STAGE_DEFAULT_FRAME

    def process_frame(self, frame_id):
        """
        use 5-point algorithm and apply homogeneous transformation

        :param frame_id:
        :return:
        """
        self.px_ref, self.px_cur = feature_tracking(
            self.last_frame, self.new_frame, self.px_ref)
        # e_mat, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp, method=cv2.RANSAC,
        #                                    prob=0.999, threshold=1.0)
        e_mat, mask = cv2.findEssentialMat(
            self.px_cur, self.px_ref, cameraMatrix=self.cam.get_camera_mat(), method=cv2.RANSAC,
            prob=0.999, threshold=1.0)
        # 5-point algorithm
        _, rot_mat, t, mask = cv2.recoverPose(
            e_mat, self.px_cur, self.px_ref, focal=self.focal, pp=self.pp)

        # homogeneous transformation
        absolute_scale = self.get_absolute_scale(frame_id)
        if absolute_scale > 0.1:
            self.cur_t = self.cur_t + absolute_scale * self.cur_R.dot(t)
            self.cur_R = rot_mat.dot(self.cur_R)

        # get additional feature points if remaining are not enough
        if self.px_ref.shape[0] < config["kMinNumFeature"]:
            self.px_cur = self.detector.detect(self.new_frame)
            self.px_cur = np.array(
                [x.pt for x in self.px_cur], dtype=np.float32)

        self.px_ref = self.px_cur

    def update(self, img, frame_id):
        """
        update position and pose

        :param img: current video image
        :param frame_id:
        """
        is_gray = img.ndim == 2
        is_same_size = img.shape[0] == self.cam.height and img.shape[1] == self.cam.width
        assert (is_gray and is_same_size), \
            "Frame: provided image has not the same size as the camera model or image is not grayscale"

        self.new_frame = img
        if self.frame_stage == STAGE_DEFAULT_FRAME:
            self.process_frame(frame_id)
        elif self.frame_stage == STAGE_SECOND_FRAME:
            self.process_second_frame()
        elif self.frame_stage == STAGE_FIRST_FRAME:
            self.process_first_frame()
        self.last_frame = self.new_frame
