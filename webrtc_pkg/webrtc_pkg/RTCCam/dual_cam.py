from threading import Thread
from rtcbot import CVCamera, CVDisplay

import cv2
import time
import asyncio
import numpy as np

mode = "1"


class WebCam(object):
    def __init__(self, src, width, height):
        self._width = width
        self._height = height

        # TODO: set wid/heigth in Constructor
        # Then, Check performance of them
        self._capture = cv2.VideoCapture(src)
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)

    def getFrame(self):
        _, self.frame = self._capture.read()
        return self.frame

    def stop(self):
        self._capture.release()


class CSICam(WebCam):
    def __init__(self, src, width, height, flip):
        self._cameranumber = src
        self._flip = flip
        self._width = width
        self._height = height

        self._pipeline = self.gstreamer_pipeline()

        self._capture = cv2.VideoCapture(self._pipeline, cv2.CAP_GSTREAMER)

    def gstreamer_pipeline(
        self,
        capture_width=1280,
        capture_height=720,
        framerate=60,
    ):
        return (
            "nvarguscamerasrc sensor_id=%d ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! "
            "appsink"
            % (
                self._cameranumber,
                capture_width,
                capture_height,
                framerate,
                self._flip,
                self._width,
                self._height,
            )
        )


class DualCam(CVCamera):
    def __init__(self, cam_num1, cam_num2, width, height, flip1=2, flip2=2):
        self._flip1 = flip1
        self._flip2 = flip2

        self._cam_num1 = cam_num1
        self._cam_num2 = cam_num2

        self._dispW = width
        self._dispH = height

        self._mode = True
        self._cnt = 0

        self._loop = asyncio.get_event_loop()

        super().__init__(self._dispW, self._dispH, cam_num1, 60, lambda x: x)

    def _producer(self):
        self._cam1 = WebCam(self._cam_num1, self._dispW, self._dispH)
        self._cam2 = WebCam(self._cam_num2, self._dispW, self._dispH)

        while not self._shouldClose:
            if self._mode:
                frame = self._cam1.getFrame()
            else:
                frame = self._cam2.getFrame()

            self._put_nowait(frame)

        self._cam1.stop()
        self._cam2.stop()
        self._setReady(False)

    def set_mode(self, mode):
        self._mode = mode

    def close(self):
        self._cam1.stop()
        self._cam2.stop()
        super().close()


class DualCSICam(DualCam):
    def __init__(self, cam_num1, cam_num2, width, height, flip1=2, flip2=0):
        super().__init__(cam_num1, cam_num2, width, height, flip1, flip2)

    def _producer(self):
        # CSICam must located on here, Other It'll cause SegFault
        self._cam1 = CSICam(self._cam_num1, self._dispW, self._dispH, self._flip1)
        self._cam2 = CSICam(self._cam_num2, self._dispW, self._dispH, self._flip2)

        while not self._shouldClose:
            if self._mode:
                frame = self._cam1.getFrame()
            else:
                frame = self._cam2.getFrame()

            self._put_nowait(frame)

        self._cam1.stop()
        self._cam2.stop()
        self._setReady(False)


if __name__ == "__main__":

    # camera = DualCam(cam_num1=0, cam_num2=4, width=640, height=480)
    camera = DualCSICam(cam_num1=0, cam_num2=1, width=640, height=480, flip1=2, flip2=0)
    display = CVDisplay()

    frameSubscription = camera.subscribe()
    display.putSubscription(frameSubscription)
    loop = asyncio.get_event_loop()

    def get_input():
        mode = True if input(f"Mode you want : ") == "2" else False
        camera.set_mode(mode)

    async def input_loop():
        while True:
            await loop.run_in_executor(None, get_input)

    try:
        asyncio.ensure_future(input_loop())
        loop.run_forever()
    finally:
        camera.close()
        display.close()
