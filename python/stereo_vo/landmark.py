"""
ランドマーククラス
"""

landmark_idx = 0


class Landmark:
    """
    ランドマーク
    """

    def __init__(self, current_pos, point_pos):
        global landmark_idx
        self.id = landmark_idx
        landmark_idx += 1
        self.abs_pos
