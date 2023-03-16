import cv2
import libcamera
import msgpack
import numpy as np

from cscore import CameraServer
from ntcore import NetworkTableInstance
from picamera2 import Picamera2
from pupil_apriltags import Detector
import imutils
import math


class RetroFinder:

    def __init__(self, camera_params, threshold, threshmode):
        self.threshold = threshold
        self.threshmode = threshmode
        self.scale_factor = 1
        self.width = camera_params[0]
        self.height = camera_params[1]
        self.tape_height = 10.5
        self.draw = False
        self.theta = 0
        self.at_detector = Detector(families="tag16h5")
        self.output_stream = CameraServer.putVideo("Processed", self.width, self.height)

    def find(self, img):
        _, thresh = cv2.threshold(img, self.threshold, 255, self.threshmode)
        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        tapes = {}
        tapes["tapes"] = []

        for c in contours:
            mmnts = cv2.moments(c)
            _, _, _, cnt_height = cv2.boundingRect(c)

            cX = int(mmnts["m10"] / mmnts["m00"]) - self.width/2
            cY = int(mmnts["m01"] / mmnts["m00"]) - self.height/2

            translation_x = cX * \
                (self.tape_height*math.cos(self.theta)/cnt_height)
            translation_y = cY * \
                (self.tape_height*math.cos(self.theta)/cnt_height)
            translation_z = (self.tape_height*self.scale_factor*math.cos(self.theta))/(cnt_height)

            tape = [translation_x, translation_y, translation_z]

            tapes["tapes"].append(
                {
                    "pose_t": tape.toList()
                }
            )

            if (self.draw):
                self.draw_result(img, c, cX, cY, tape)
        return tapes

    def draw_result(img, cnt, cX, cY, tape):
        float_formatter = {"float_kind": lambda x: f"{x:4.1f}"}
        wpi_t = np.array([tape[2], -tape[0], -tape[1]])
        cv2.drawContours(img, [cnt], -1, (0, 255, 0), 2)
        cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)
        cv2.putText(img, f"t: {np.array2string(wpi_t.flatten(), formatter=float_formatter)}", (cX - 20, cY - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    def analyze(self, request):
        buffer = request.make_buffer("lores")
        y_len = self.width * self.height
        img = np.frombuffer(buffer, dtype=np.uint8, count=y_len)
        img = img.reshape((self.height, self.width))
        img = img[: self.height, : self.width]
        self.find(img)
        self.output_stream.put(img)

def main():
    print("main")
    fullwidth = 1664
    fullheight = 1232
    width = 832
    height = 616
    # option 3: tiny, trade speed for detection distance; two circles, three squraes, ~40ms
    # width=448
    # height=308
    # fast path
    # width=640
    # height=480
    # medium crop
    # width=1920
    # height=1080

    camera = Picamera2()
    camera_config = camera.create_still_configuration(
    # one buffer to write, one to read, one in between so we don't have to wait
    buffer_count=6,
    main={
        "format": "YUV420",
        "size": (fullwidth, fullheight),
    },
    lores={"format": "YUV420", "size": (width, height)},
    controls={
        "FrameDurationLimits": (5000, 33333),  # 41 fps
        # noise reduction takes time
        "NoiseReductionMode": libcamera.controls.draft.NoiseReductionModeEnum.Off,
    },
)
    print("REQUESTED")
    print(camera_config)
    camera.align_configuration(camera_config)
    print("ALIGNED")
    print(camera_config)
    camera.configure(camera_config)
    print(camera.camera_controls)

    # Roborio IP: 10.1.0.2
    # Pi IP: 10.1.0.21
    camera_params = [width, height]
    output = RetroFinder(camera_params, 245, cv2.THRESH_BINARY)

    camera.start()
    try:
        while True:
            request = camera.capture_request()
            try:
                output.analyze(request)
            finally:
                request.release()
    finally:
        camera.stop()
main()
