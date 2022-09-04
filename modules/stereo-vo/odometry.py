from typing import Tuple
import operator
import numpy as np
import cv2


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

        key_points_1 = [key_points_1[match.queryIdx] for match in matches]
        key_points_2 = [key_points_2[match.trainIdx] for match in matches]
        return key_points_1, key_points_2


def feature_tracking(pre_image, current_image, key_points_1):
    """
    calc optical flow

    :param pre_image: previous image mat
    :param current_image: current image mat
    :param key_points_1: feature points of previous image
    :return: start points and end points of optical flow
    """
    # parameters for optical flow
    lk_params = dict(winSize=(21, 21),
                     # maxLevel = 3,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    # shape: [k,2] [k,1] [k,1]
    key_points_2, statuses, errors = cv2.calcOpticalFlowPyrLK(
        pre_image, current_image, prevPts=key_points_1, nextPts=None, **lk_params)
    # bool list for optical flow (same length with px_ref)
    statuses = statuses.reshape(statuses.shape[0])
    # extract feature points as start points of optical flows
    key_points_1 = key_points_1[statuses == 1]

    # extract feature points as end points of optical flows
    key_points_2 = key_points_2[statuses == 1]
    return key_points_1, key_points_2


# minimum keypoint number to refresh
kMinNumFeature = 1500
