config = {
    "root_path": "/mnt/d/datasets/KITTI/odometry",
    "to_track": {
        "maxCorners": 100,
        "qualityLevel": 0.3,
        "minDistance": 7,
        "blockSize":7
    },
    "stereo_sgbm": {
        "minDisparity": 0,
        "numDisparities": 96,
        "blockSize": 9,
        "P1": 8 * 9 * 9,
        "P2": 32 * 9 * 9,
        "disp12MaxDiff": 1,
        "preFilterCap": 63,
        "uniquenessRatio": 10,
        "speckleWindowSize": 100,
        "speckleRange": 32
    },
    "disparity_min": 10,
    "disparity_max": 96
}