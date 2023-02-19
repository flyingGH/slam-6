import cv2

config = {
    # データセットのルートパス
    "root_path": "/mnt/d/datasets/KITTI/odometry",
    "seq_id": "00",
    "img_id_end": 4541,
    # カメラ内部パラメータ
    "inner_params": {
        "width": 1241.0,
        "height": 376.0,
        "fx": 718.8560,
        "fy": 718.8560,
        "cx": 607.1928,
        "cy": 185.2157
    },
    # Optical Flow のパラメータ
    "lk_params": {
        "winSize": (21, 21),
        # maxLevel = 3,
        "criteria": (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
    },
    # keypoint 再検出の閾値
    "kMinNumFeature": 1500
}
