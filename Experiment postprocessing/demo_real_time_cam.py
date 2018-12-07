from pose_tracker_and_helpers import FrameProcessor
from config import AllParams


def run_demo():
    if False:
        for t in ["2018-12-06 13-52-19", "2018-12-06 14-08-07", "2018-12-06 14-29-17", "2018-12-06 14-39-39"]:
            params = AllParams("../DataCollection/data/{t}/cam_1_{t}.mp4".format(t=t))
            fp = FrameProcessor(params)
            fp.run_demo()
    else:
        import os

        for root, subdirs, files in os.walk("../TotalCapture"):
            if len(subdirs) == 0:  # Deepest level, here's where the videos to analyze are
                for f in files:
                    name, ext = os.path.splitext(f)
                    if ext == '.mp4' and not name.endswith('_rendered'):  # Only process source mp4 videos (ignore rendered output ones)
                        if os.path.exists(os.path.join(root, "{}.h5".format(name))):
                            print("Ignoring '{}' because it's already been processed. Next!".format(os.path.join(root, "{}.h5".format(name))))
                            continue
                        params = AllParams(os.path.join(root, f))
                        fp = FrameProcessor(params)
                        fp.run_demo()

if __name__ == '__main__':
    run_demo()
