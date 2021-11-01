import av  # PyAV 임포트
import cv2
import gi
import time
import logging
import asyncio
import numpy as np

from rtcbot import CVCamera, CVDisplay

gi.require_version("Gst", "1.0")
from gi.repository import GObject, Gst

# TODO: Where should place this?
Gst.init(None)


class WebCam(CVCamera):
    # TODO: set cam number 0, 1 etc...
    _log = logging.getLogger("rtcbot.WebCam")

    def __init__(
        self,
        width=640,  # 320,
        height=480,  # 240,
        camID=0,
        fps=30,
        preprocessframe=lambda x: x,
        loop=None,
    ):

        self._width = width
        self._height = height
        self._cameranumber = camID
        self._fps = fps
        self._processframe = preprocessframe

        self._is_camera_on = False

        super().__init__(
            self._width, self._height, self._cameranumber, self._fps, self._processframe
        )

    def _producer(self):
        """
        Runs the actual frame capturing code.
        """

        cap = cv2.VideoCapture(self._cameranumber)

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        cap.set(cv2.CAP_PROP_FPS, self._fps)

        if self._is_camera_on == False:
            ret, frame = cap.read()
            if not ret:
                self._log.error("Camera Read Failed %s", str(ret))
                cap.release()
                self._setError(ret)
                return
            else:
                self._is_camera_on = True
                self._log.debug("Camera Ready")

        self._setReady(True)
        while not self._shouldClose:
            ret, frame = cap.read()
            if not ret:
                self._log.error("CV read error %s", str(ret))
            else:
                # This optional function is given by the user. default is identity x->x
                # frame = self._processframe(frame)

                # Send the frame to all subscribers
                self._put_nowait(frame)

        cap.release()
        self._setReady(False)

class CSICam(CVCamera):
    """
    GSTCam For Jetson Nano
    """

    _log = logging.getLogger("rtcbot.CSICam")

    def __init__(
        self,
        width=640,
        height=480,
        camID=0,
        fps=60,
        flip_method=2,
        preprocessframe=lambda x: x,
        loop=None,
        capture_mode="CV",
    ):

        self._width = width
        self._height = height
        self._cameranumber = camID
        self._fps = fps
        self._flip_method = flip_method

        self._processframe = preprocessframe
        self._capture_mode = capture_mode

        self._is_camera_on = False

        super().__init__(
            self._width, self._height, self._cameranumber, self._fps, self._processframe
        )

    def gstreamer_pipeline(
        self,
        capture_width=1280,
        capture_height=720,
        framerate=60,
        flip_method=2,
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
                self._fps,
                flip_method,
                self._width,
                self._height,
            )
        )

    def gst_to_opencv(self, sample):
        buf = sample.get_buffer()
        caps = sample.get_caps()

        # print(caps.get_structure(0).get_value("format"))
        # print(caps.get_structure(0).get_value("height"))
        # print(caps.get_structure(0).get_value("width"))

        # print(buf.get_size())

        arr = np.ndarray(
            (
                caps.get_structure(0).get_value("height"),
                caps.get_structure(0).get_value("width"),
                3,
            ),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=np.uint8,
        )
        return arr

    def _producer(self):
        """
        Runs the actual frame capturing code.
        """

        gst_cmd = self.gstreamer_pipeline(
            capture_width=1280, capture_height=720, flip_method=self._flip_method
        )
        print(gst_cmd)

        if self._capture_mode == "GST":
            pipeline = Gst.parse_launch(gst_cmd)
            sink = pipeline.get_by_name("sink")
            pipeline.set_state(Gst.State.PLAYING)
        elif self._capture_mode == "CV":
            print("CV Mode")
            cap = cv2.VideoCapture(gst_cmd, cv2.CAP_GSTREAMER)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            cap.set(cv2.CAP_PROP_FPS, self._fps)

            if self._is_camera_on == False:
                ret, frame = cap.read()
                if not ret:
                    self._log.error("Camera Read Failed %s", str(ret))
                    cap.release()
                    self._setError(ret)
                    return
                else:
                    self._is_camera_on = True
                    self._log.debug("Camera Ready")

        self._setReady(True)
        while not self._shouldClose:
            if self._capture_mode == "GST":
                sample = sink.emit("pull-sample")
                if not sample:
                    continue
                    self._log.error("GST read error")
                else:
                    new_frame = self.gst_to_opencv(sample)
                    self._put_nowait(new_frame)

            elif self._capture_mode == "CV":
                ret, frame = cap.read()
                if not ret:
                    self._log.error("CV read error %s", str(ret))
                else:
                    self._put_nowait(frame)

        if self._capture_mode == "CV":
            cap.release()

        pipeline.set_state(Gst.State.NULL)
        self._setReady(False)
        self._log.info("Ended camera capture")


class GSTCam(CVCamera):
    """
    Uses a camera supported by OpenCV.

    When initializing, can give an optional function which preprocesses frames as they are read, and returns the
    modified versions thereof. Please note that the preprocessing happens synchronously in the camera capture thread,
    so any processing should be relatively fast, and should avoid pure python code due to the GIL. Numpy and openCV functions
    should be OK.
    """

    _log = logging.getLogger("rtcbot.GSTCam")

    def __init__(
        self,
        width=640,
        height=480,
        camID=0,
        fps=30,
        preprocessframe=lambda x: x,
        loop=None,
    ):

        self._width = width
        self._height = height
        self._cameranumber = camID
        self._fps = fps
        self._processframe = preprocessframe

        self._is_camera_on = False

        super().__init__(
            self._width, self._height, self._cameranumber, self._fps, self._processframe
        )

    def gstreamer_pipeline(
        self,
        capture_width=640,
        capture_height=480,
        display_width=640,
        display_height=480,
        framerate=30,
        flip_method=0,
    ):

        return (
            "v4l2src device=/dev/video%d ! "
            "videoconvert ! videorate ! "
            "video/x-raw, framerate=%d/1, width=%d, height=%d, format=(string)BGR ! "
            "videoconvert ! "
            "appsink sync=false max-buffers=2 drop=true name=sink emit-signals=true"
            % (
                self._cameranumber,
                self._fps,
                self._width,
                self._height,
            )
        )

    def gst_to_opencv(self, sample):
        buf = sample.get_buffer()
        caps = sample.get_caps()

        # print(caps.get_structure(0).get_value("format"))
        # print(caps.get_structure(0).get_value("height"))
        # print(caps.get_structure(0).get_value("width"))

        # print(buf.get_size())

        arr = np.ndarray(
            (
                caps.get_structure(0).get_value("height"),
                caps.get_structure(0).get_value("width"),
                3,
            ),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=np.uint8,
        )
        return arr

    def _producer(self):
        """
        Runs the actual frame capturing code.
        """

        gst_cmd = self.gstreamer_pipeline()
        print(gst_cmd)
        pipeline = Gst.parse_launch(gst_cmd)

        sink = pipeline.get_by_name("sink")
        pipeline.set_state(Gst.State.PLAYING)

        # if self._is_camera_on == False:
        #     ret, frame = cap.read()
        #     if not ret:
        #         self._log.error("Camera Read Failed %s", str(ret))
        #         cap.release()
        #         self._setError(ret)
        #         return
        #     else:
        #         self._is_camera_on = True
        #         self._log.debug("Camera Ready")

        # t = time.time()
        # i = 0
        # self._setReady(True)
        # print("Done...")
        while not self._shouldClose:
            sample = sink.emit("pull-sample")
            # ret, frame = cap.read()
            if not sample:
                continue
                # self._log.error("GST read error")
            else:
                # This optional function is given by the user. default is identity x->x

                new_frame = self.gst_to_opencv(sample)

                # cv2.imshow("frame", new_frame)
                # if cv2.waitKey(1) & 0xFF == ord("q"):
                #     break

                # Send the frame to all subscribers
                self._put_nowait(new_frame)

        # cap.release()
        pipeline.set_state(Gst.State.NULL)
        self._setReady(False)
        self._log.info("Ended camera capture")


class GSTCamH264(CVCamera):
    _log = logging.getLogger("rtcbot.GSTCam")

    def __init__(
        self,
        width=640,
        height=480,
        camID=0,
        fps=30,
        preprocessframe=lambda x: x,
        loop=None,
    ):

        self._width = width
        self._height = height
        self._cameranumber = camID
        self._fps = fps
        self._processframe = preprocessframe

        self._is_camera_on = False

        super().__init__(
            self._width, self._height, self._cameranumber, self._fps, self._processframe
        )

    def gstreamer_pipeline(
        self,
        capture_width=640,
        capture_height=480,
        display_width=640,
        display_height=480,
        framerate=30,
        flip_method=0,
    ):

        return (
            "v4l2src device=/dev/video%d ! "
            "videoconvert ! videorate ! "
            "video/x-raw, framerate=%d/1, width=%d, height=%d ! "
            "videoconvert ! x264enc tune=zerolatency ! "
            "appsink sync=false max-buffers=2 drop=true name=sink emit-signals=true"
            % (
                self._cameranumber,
                self._fps,
                self._width,
                self._height,
            )
        )

    def gst_parse(self, sample):
        buf = sample.get_buffer()
        caps = sample.get_caps()

        # arr = np.ndarray(
        #     (
        #         caps.get_structure(0).get_value("height"),
        #         caps.get_structure(0).get_value("width"),
        #         3,
        #     ),
        #     buffer=buf.extract_dup(0, buf.get_size()),
        #     dtype=np.uint8,
        # )

        arr = buf.extract_dup(0, buf.get_size())

        return arr

    def _producer(self):
        """
        Runs the actual frame capturing code.
        """

        gst_cmd = self.gstreamer_pipeline()
        print(gst_cmd)
        pipeline = Gst.parse_launch(gst_cmd)

        sink = pipeline.get_by_name("sink")
        pipeline.set_state(Gst.State.PLAYING)

        while not self._shouldClose:
            sample = sink.emit("pull-sample")
            # ret, frame = cap.read()
            if not sample:
                continue
                # self._log.error("GST read error")
            else:
                new_frame = self.gst_parse(sample)
                
                # Send the frame to all subscribers
                self._put_nowait(new_frame)

        # cap.release()
        pipeline.set_state(Gst.State.NULL)
        self._setReady(False)
        self._log.info("Ended camera capture")


class RawCam(CVCamera):
    """
    Uses a camera supported by OpenCV.

    When initializing, can give an optional function which preprocesses frames as they are read, and returns the
    modified versions thereof. Please note that the preprocessing happens synchronously in the camera capture thread,
    so any processing should be relatively fast, and should avoid pure python code due to the GIL. Numpy and openCV functions
    should be OK.
    """

    _log = logging.getLogger("rtcbot.RawCam")

    def __init__(
        self,
        width=640,
        height=480,
        camID=0,
        fps=30,
        preprocessframe=lambda x: x,
        loop=None,
    ):

        self._width = width
        self._height = height
        self._cameranumber = camID
        self._fps = fps
        self._processframe = preprocessframe

        self._is_camera_on = False

        self._container = av.open(
            f"/dev/video{self._cameranumber}",
            options={"s": "1280x720", "framerate": "60"},
        )
        self._video = self._container.streams.video[0]

        super().__init__(
            self._width, self._height, self._cameranumber, self._fps, self._processframe
        )

    def _producer(self):
        """
        Runs the actual frame capturing code.
        """
        frames = self._container.decode(video=0)

        while not self._shouldClose:
            frame = next(frames)
            img = frame.to_image()
            self._put_nowait(img)

        self._log.info("Ended camera capture")


if __name__ == "__main__":
    # camera = WebCam(camID=0)
    camera = CSICam(camID=0)
    # camera = GSTCam(camID=0)
    # camera = RawCam(camID=0)
    display = CVDisplay()

    frameSubscription = camera.subscribe()
    display.putSubscription(frameSubscription)

    try:
        asyncio.get_event_loop().run_forever()
    finally:
        camera.close()
        display.close()
