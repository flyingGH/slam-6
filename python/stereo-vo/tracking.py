import copy
from typing import Tuple, Union
import operator
import numpy as np
import cv2

_detector = cv2.ORB_create()
_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)


def feature_tracking(image_1: np.ndarray, image_2: np.ndarray,
                     key_points_1: Union[cv2.KeyPoint, None], draw: bool) -> Tuple[np.ndarray, np.ndarray]:
    """
    calc matched keypoints

    :param image_1: previous image mat
    :param image_2: current image mat
    :param key_points_1: feature points of previous image
    :param draw: draw results or not
    :return: start points and end points of optical flow
    """
    if not key_points_1:
        key_points_1, descriptors_1 = _detector.detectAndCompute(image_1, None)
    else:
        descriptors_1 = _detector.compute(image_1, key_points_1)
    key_points_2, descriptors_2 = _detector.detectAndCompute(image_2, None)

    # left_image_w_key_points = cv2.drawKeypoints(image_1, key_points_1,
    #                                             outImage=None, color=(0, 255, 0), flags=0)

    matches = list(_matcher.match(descriptors_1, descriptors_2))

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

    key_points_1 = np.array([key_points_1[match.queryIdx].pt for match in matches])
    key_points_2 = np.array([key_points_2[match.trainIdx].pt for match in matches])
    return key_points_1, key_points_2


def direct_tracking(image_1: np.ndarray, image_2: np.ndarray,
                    key_points_1: Union[np.ndarray, None], draw: bool) -> Tuple[np.ndarray, np.ndarray]:
    """
    calc optical flow

    :param image_1: previous image mat
    :param image_2: current image mat
    :param key_points_1: feature points of previous image
    :param draw: draw results or not
    :return: start points and end points of optical flow
    """
    if not key_points_1:
        # params for Shi-Tomasi corner detection
        feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
        key_points_1 = cv2.goodFeaturesToTrack(image_1, mask=None, **feature_params)
        key_points_1 = np.array([point[0] for point in key_points_1])
    # parameters for optical flow
    lk_params = dict(winSize=(21, 21),
                     # maxLevel = 3,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    # shape: [k,2] [k,1] [k,1]
    key_points_2, statuses, errors = cv2.calcOpticalFlowPyrLK(
        image_1, image_2, prevPts=key_points_1, nextPts=None, **lk_params)
    # bool list for optical flow (same length with px_ref)
    statuses = statuses.reshape(statuses.shape[0])
    # extract feature points as start points of optical flows
    key_points_1 = key_points_1[statuses == 1]
    # extract feature points as end points of optical flows
    key_points_2 = key_points_2[statuses == 1]

    if draw:
        mask = np.zeros_like(cv2.cvtColor(image_1, cv2.COLOR_GRAY2BGRA))
        color = np.random.randint(0, 255, (100, 3))
        # draw the tracks
        result_image = cv2.cvtColor(copy.deepcopy(image_2), cv2.COLOR_GRAY2BGRA)
        for i, (new, old) in enumerate(zip(key_points_2, key_points_1)):
            a, b = new.ravel()
            c, d = old.ravel()
            mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
            result_image = cv2.circle(result_image, (int(a), int(b)), 5, color[i].tolist(), -1)
        result_image = cv2.add(result_image, mask)
        cv2.imshow("optical flow", result_image)
        cv2.waitKey()
        cv2.destroyAllWindows()

    return key_points_1, key_points_2
