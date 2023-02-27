# pylint: disable=missing-module-docstring
# pylint: disable=missing-function-docstring
# pylint: disable=missing-class-docstring
# pylint: disable=import-error
import time

import cv2
import libcamera
import msgpack
import numpy as np

from cscore import CameraServer
from ntcore import NetworkTableInstance
from picamera2 import Picamera2
from pupil_apriltags import Detector


class TagFinder:
    def __init__(self, width, height):
        self.frame_time = time.time()
        # each camera has its own topic
        self.topic_name = getserial()  # was: topic_name = "tags"
        self.width = width
        self.height = height

        self.initialize_nt()

        # tag size was wrong before.  full size is 0.2m but
        # https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide
        # this points out that the apriltag library expects tag_size to be
        # the boundary between the outer white boundary and the inner black
        # boundary, which in this case is 0.15 m
        self.tag_size = 0.15
        # self.circle_tag_size = 0.8
        self.at_detector = Detector(families="tag16h5")
        # self.at_circle_detector = Detector(families="tagCircle21h7")
        # self.output_stream = CameraServer.putVideo("Processed", width, height)
        # vertical slice
        # TODO: one slice for squares one for circles
        # for now use the full frame
        #        self.output_stream = CameraServer.putVideo("Processed", width, int(height / 2))
        self.output_stream = CameraServer.putVideo("Processed", width, height)
        # self.camera_params = [
        #     666,
        #     666,
        #     width / 2,
        #     height / 2,
        # ]
        self.camera_params = [
            666,
            666,
            width,
            height,
        ]

    def analyze(self, request):
        buffer = request.make_buffer("lores")
        #        buffer = request.make_buffer("main")
        metadata = request.get_metadata()
        # print(metadata)
        # sensor timestamp is the boottime when the first byte was received from the sensor
        sensor_timestamp = metadata[
            "SensorTimestamp"
        ]  # pylint: disable=unused-variable
        system_time_ns = time.clock_gettime_ns(
            time.CLOCK_BOOTTIME
        )  # pylint: disable=no-member, unused-variable
        # time_delta_ns = system_time_ns - sensor_timestamp
        # print(sensor_timestamp, system_time_ns, time_delta_ns//1000000) # ms

        start_time = time.time()
        y_len = self.width * self.height
        # truncate, ignore chrominance
        # this makes a view, takes 300ns
        img = np.frombuffer(buffer, dtype=np.uint8, count=y_len)
        # this also makes a view, takes 150ns
        img = img.reshape((self.height, self.width))
        # slice out the middle, there's never a target near the top or the bottom
        # TODO: one slice for squares one for circles
        # this also makes a view, takes 150ns
        # for now use the full frame
        #        img = img[int(self.height / 4) : int(3 * self.height / 4), : self.width]
        img = img[: self.height, : self.width]
        # for debugging the size:
        # img = np.frombuffer(buffer, dtype=np.uint8)
        # img = img.reshape((int(height*3/2), -1))
        # img = img[: height, ]
        # print(img.shape) # (self.width,self.height) = (616,832)
        # print("MIN", np.amin(img), "MAX", np.amax(img))

        # # not instant, ~300us
        # led_on = np.amax(img) > 200
        # self.vision_nt_led.set(led_on)

        result = self.at_detector.detect(
            img,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size,
        )
        self.draw_result(img, result)

        tags = {}
        tags["tags"] = []

        for result_item in result:
            if result_item.hamming > 0:
                continue

            tags["tags"].append(
                {
                    "id": result_item.tag_id,
                    "pose_t": result_item.pose_t.tolist(),
                    "pose_R": result_item.pose_R.tolist(),
                }
            )

        current_time = time.time()
        analysis_et = current_time - start_time
        total_et = current_time - self.frame_time

        tags["et"] = total_et
        # print(tags)

        posebytes = msgpack.packb(tags)

        self.vision_nt_msgpack.set(posebytes)

        fps = 1 / total_et
        self.frame_time = current_time
        self.draw_text(img, f"analysis ET(ms) {1000*analysis_et:.0f}", (5, 25))
        self.draw_text(img, f"total ET(ms) {1000*total_et:.0f}", (5, 65))
        self.draw_text(img, f"fps {fps:.1f}", (5, 105))

        self.output_stream.putFrame(img)

    def draw_result(self, image, result):
        for result_item in result:
            if result_item.hamming > 0:
                continue
            (pt_a, pt_b, pt_c, pt_d) = result_item.corners
            pt_a = (int(pt_a[0]), int(pt_a[1]))
            pt_b = (int(pt_b[0]), int(pt_b[1]))
            pt_c = (int(pt_c[0]), int(pt_c[1]))
            pt_d = (int(pt_d[0]), int(pt_d[1]))

            cv2.line(image, pt_a, pt_b, (255, 255, 255), 2)
            cv2.line(image, pt_b, pt_c, (255, 255, 255), 2)
            cv2.line(image, pt_c, pt_d, (255, 255, 255), 2)
            cv2.line(image, pt_d, pt_a, (255, 255, 255), 2)

            (c_x, c_y) = (int(result_item.center[0]), int(result_item.center[1]))
            cv2.circle(image, (c_x, c_y), 10, (255, 255, 255), -1)

            tag_id = result_item.tag_id
            self.draw_text(image, f"id {tag_id}", (c_x, c_y))
            tag_family = result_item.tag_family.decode("utf-8")
            self.draw_text(image, f"id {tag_family}", (c_x, c_y + 40))

            # put the pose translation in the image
            # the use of 'item' here is to force a scalar to format
            if result_item.pose_t is not None:
                self.draw_text(
                    image, f"X {result_item.pose_t.item(0):.2f}m", (c_x, c_y + 80)
                )
                self.draw_text(
                    image, f"Y {result_item.pose_t.item(1):.2f}m", (c_x, c_y + 120)
                )
                self.draw_text(
                    image, f"Z {result_item.pose_t.item(2):.2f}m", (c_x, c_y + 160)
                )

    # these are white with black outline
    def draw_text(self, image, msg, loc):
        cv2.putText(image, msg, loc, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 6)
        cv2.putText(image, msg, loc, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

    def initialize_nt(self):
        """Start NetworkTables with Rio as server, set up publisher."""
        inst = NetworkTableInstance.getDefault()
        inst.startClient4("tag-finder")
        # this is always the RIO IP address; set a matching static IP on your
        # laptop if you're using this in simulation.
        inst.setServer("10.1.0.2")
        # Table for vision output information
        self.vision_nt = inst.getTable("Vision")
        self.vision_nt_msgpack = self.vision_nt.getRawTopic(self.topic_name).publish(
            "msgpack"
        )

    def reconnect_nt(self):
        inst = NetworkTableInstance.getDefault()
        inst.stopClient() # without this, reconnecting doesn't work
        inst.startClient4("tag-finder")

def getserial():
    with open("/proc/cpuinfo", "r", encoding="ascii") as cpuinfo:
        for line in cpuinfo:
            if line[0:6] == "Serial":
                return line[10:26]
    return ""


def main():
    print("main")

    # full frame, 2x2, to set the detector mode
    fullwidth = 1664  # slightly larger than the detector, to match stride
    fullheight = 1232
    # fast path, 200fps, narrow crop
    # fullwidth = 640
    # fullheight = 480
    # medium crop
    # fullwidth = 1920
    # fullheight = 1080

    # lores for apriltag detector
    # option 1: big, all the pixels yields 270ms delay, but seems even higher
    # docs say 41 fps is possible
    # width=1664
    # height=1232
    # option 2: medium, two circles, three squares, timer says ~80ms but seems more like 500ms
    # remember to note this delay in the kalman filter input
    # TODO: report the actual delay in network tables
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
    # don't use video, it tries to process all the frames?
    # camera_config = camera.create_video_configuration(
    # no need for extra buffers, dropping frames is fine
    #     buffer_count=4,
    #     encode="lores",
    # use single-frame
    camera_config = camera.create_still_configuration(
        # one buffer to write, one to read, one in between so we don't have to wait
        buffer_count=6,
        main={
            "format": "YUV420",
            "size": (fullwidth, fullheight),
        },
        lores={"format": "YUV420", "size": (width, height)},
        controls={
            # fast shutter means more gain
            # "AnalogueGain": 8.0,
            # the flashing LED makes the Auto-Exposure freak out, so turn it off:
            # "ExposureTime": 5000,
            # go as fast as possible but no slower than 30fps
            # "FrameDurationLimits": (100, 33333),
            "FrameDurationLimits": (24000, 33333),  # 41 fps
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

    output = TagFinder(width, height)
    # the callback blocks the camera thread, so don't do that.
    # camera.pre_callback = output.pre_callback
    camera.start()
    frame_counter = 0
    try:
        while True:
            # periodically reconnect to NT.
            # without these attempts, network disruption results in 
            # permanent disconnection
            frame_counter += 1
            if frame_counter > 20:
                print("RECONNECTING")
                frame_counter = 0
                output.reconnect_nt()
            # the most recent frame, maybe from the past
            request = camera.capture_request()
            try:
                output.analyze(request)
            finally:
                # the frame is owned by the camera so remember to release it
                request.release()
            # time.sleep(1)
    finally:
        # not in video mode
        # camera.stop_recording()
        camera.stop()


main()
