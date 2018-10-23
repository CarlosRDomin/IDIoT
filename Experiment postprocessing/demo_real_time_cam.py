from pose_tracker_and_helpers import FrameProcessor
from config import AllParams


def run_demo():
    params = AllParams("../DataCollection/data/2018-09-18 11-43-17/cam_1_2018-09-18 11-43-17.mp4")
    fp = FrameProcessor(params)
    fp.run_demo()


if __name__ == '__main__':
    run_demo()
