"""
 This file contains helper classes that offload some computation to a separate process, for better performance.
"""
import cv2
from multiprocessing import Process, Pipe
from datetime import datetime


class ProcessImshow:
    """
     Helper class that allows rendering images (cv2.imshow) *in a separate process* by constantly loading and displaying an image specified by its file name.
    """

    def __init__(self, file_name=None, win_name='Output'):
        self.file_name = file_name
        self.win_name = win_name
        self.lastT = datetime.now()

    def __call__(self, pipe):
        self.pipe = pipe
        cv2.namedWindow(self.win_name)
        cv2.moveWindow(self.win_name, 0, 0)
        cv2.waitKeyEx(1)

        while True:
            if self.file_name is not None:
                while self.pipe.poll():  # If there's available images
                    self.file_name = self.pipe.recv()  # Read their filenames (in case each frame is saved with a unique name)
                img = cv2.imread(self.file_name)
            else:
                img = None
                while self.pipe.poll():  # If there's available images
                    img = self.pipe.recv()  # Read them all
            if img is not None:
                cv2.imshow(self.win_name, img)
            key = cv2.waitKeyEx(1)
            if key >= 0:
                self.pipe.send(key)

            if False:
                print('\t\tProcessImshow: {:.2f}ms'.format((datetime.now()-self.lastT).total_seconds()*1000))
            self.lastT = datetime.now()


class ProcessImshowHelper:
    """
     Helper class responsible for creating and communicating with a ProcessImshow instance.
     That is: it spawns a new process that runs ProcessImshow, starts it, and and allows reading the keys pressed (if any) in the cv2 window
    """

    def __init__(self, *args, **kwargs):
        self.pipe, imshow_pipe = Pipe()  # Create a Pipe to communicate both processes
        self.imshower = ProcessImshow(*args, **kwargs)  # Forward args (and kwargs) to ProcessImshow
        self.imshow_process = Process(target=self.imshower, args=(imshow_pipe,))  # Spawn a new process and run the ProcessImshow in it
        self.imshow_process.daemon = True  # Kill the process when the main process is killed
        self.imshow_process.start()

    def set_filename(self, new_file_name):
        self.pipe.send(new_file_name)

    def get_filename(self):
        return self.imshower.file_name

    def get_keys_pressed(self):
        keys = []
        while self.pipe.poll():  # Check if the user has pressed any keys
            keys.append(self.pipe.recv())  # Read the key value and append it to the list

        return keys