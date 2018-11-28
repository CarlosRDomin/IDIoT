from pose_tracker_and_helpers import FrameProcessor
from config import AllParams


def run_demo():
    t = "2018-11-01 16-52-23"
    params = AllParams("../DataCollection/data/{t}/cam_1_{t}.mp4".format(t=t))
    fp = FrameProcessor(params)
    fp.run_demo()


if __name__ == '__main__':
    run_demo()
