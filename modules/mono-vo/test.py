"""
from https://github.com/uoip/monoVO-python
"""
import numpy as np
import cv2

from visual_odometry import PinholeCamera, VisualOdometry


def main():
    root_path = "/mnt/d/datasets/KITTI/odometry"
    cam = PinholeCamera(width=1241.0, height=376.0, fx=718.8560, fy=718.8560, cx=607.1928, cy=185.2157)
    vo = VisualOdometry(cam, f'{root_path}/data_odometry_poses/dataset/poses/00.txt')

    # image for trajectory drawing
    traj = np.zeros((600, 600, 3), dtype=np.uint8)

    for img_id in range(4541):
        img = cv2.imread(f'{root_path}/dataset/sequences/00/image_0/{str(img_id).zfill(6)}.png', 0)

        vo.update(img, img_id)

        cur_t = vo.cur_t
        if img_id > 2:
            cur_t = cur_t.reshape(-1)
            x, y, z = cur_t[0], cur_t[1], cur_t[2]
        else:
            x, y, z = 0., 0., 0.

        # convert to the image coordinate
        # (swap variables and add offsets)
        draw_x, draw_y = int(z) + 90, int(x) + 290
        true_x, true_y = int(vo.trueZ) + 90, int(vo.trueX) + 290

        # estimated trajectory (smaller line, green -> blue)
        cv2.circle(traj, (draw_x, draw_y), radius=1,
                   color=(img_id * 255 / 4540, 255 - img_id * 255 / 4540, 0), thickness=1)
        # true trajectory (larger line & red)
        cv2.circle(traj, (true_x, true_y), radius=1, color=(0, 0, 255), thickness=2)
        # background color (black)
        cv2.rectangle(traj, pt1=(10, 20), pt2=(600, 60), color=(0, 0, 0), thickness=-1)

        # put text (white) from upper left side of the trajectory window
        text = f"Coordinates: x={x:2f}m y={y:2f}m z={z:2f}m"
        cv2.putText(traj, text, org=(20, 40), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1,
                    color=(255, 255, 255), thickness=1, lineType=8)

        cv2.imshow('Road facing camera', img)
        cv2.imshow('Trajectory', traj)
        cv2.waitKey(1)

    cv2.imwrite('map.png', traj)


if __name__ == "__main__":
    main()
