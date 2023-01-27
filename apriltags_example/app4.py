# pylint: disable=missing-module-docstring
# pylint: disable=missing-function-docstring
# pylint: disable=missing-class-docstring

import io
import json
import time
import cv2
import numpy as np
from cscore import CameraServer
from ntcore import NetworkTableInstance
from picamera2 import MappedArray
from picamera2 import Picamera2
from pupil_apriltags import Detector

global_frame_time = time.time()

class TagFinder:
    def __init__(self, width, height):
        self.frame_time = time.time()
        self.width = width
        self.height = height
        # NetworkTables Rio as server
        # NetworkTables.initialize(server="10.1.0.14")
        # NetworkTables Pi as server (for testing)
        # NetworkTables.initialize()
        # Table for vision output information
        self.vision_nt = NetworkTableInstance.getDefault().getTable("Vision")
        # server mode for testing
        NetworkTableInstance.getDefault().startServer()
        self.vision_nt_id = self.vision_nt.getDoubleArrayTopic("id").publish()
        self.vision_nt_tx = self.vision_nt.getDoubleArrayTopic("pose_t_x").publish()
        self.vision_nt_ty = self.vision_nt.getDoubleArrayTopic("pose_t_y").publish()
        self.vision_nt_tz = self.vision_nt.getDoubleArrayTopic("pose_t_z").publish()
        self.vision_nt_rx = self.vision_nt.getStringArrayTopic("pose_R_x").publish()
        self.vision_nt_ry = self.vision_nt.getStringArrayTopic("pose_R_y").publish()
        self.vision_nt_rz = self.vision_nt.getStringArrayTopic("pose_R_z").publish()
        self.tag_size = 0.2
        self.circle_tag_size = 0.8
        self.at_detector = Detector(families="tag16h5")
        self.at_circle_detector = Detector(families="tagCircle21h7")
        # self.output_stream = CameraServer.putVideo("Processed", width, height)
        # vertical slice
        # TODO: one slice for squares one for circles
        self.output_stream = CameraServer.putVideo("Processed", width, int(height/2))
        self.camera_params = [
            357.1,
            357.1,
            width / 2,
            height / 2,
        ]

    def pre_callback(self, request):
        with MappedArray(request, "lores") as m:
            self.analyze(m.array, self.width, self.height)

    def analyze(self, data, width, height):
        start_time = time.time()

        y_len = width * height
        # truncate, ignore chrominance
        img = np.frombuffer(data, dtype=np.uint8, count=y_len)
        img = img.reshape((height, width))
        # slice out the middle, there's never a target near the top or the bottom
        # TODO: one slice for squares one for circles
        img = img[int(height/4):int(3*height/4), :width]
        # img = img[:height, :width]
        # for debugging the size:
        # img = np.frombuffer(data, dtype=np.uint8)
        # img = img.reshape((int(height*3/2), -1))
        # img = img[: height, ]
        # print(img.shape) # (width,height) = (616,832)

        result = self.at_detector.detect(
            img,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size,
        )

        circle_result = self.at_circle_detector.detect(
            img,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.circle_tag_size,
        )

        result = result + circle_result

        self.draw_result(img, result)

        id_list = []
        pose_t_x_list = []
        pose_t_y_list = []
        pose_t_z_list = []
        pose_r_x_list = []
        pose_r_y_list = []
        pose_r_z_list = []

        for result_item in result:
            if result_item.hamming > 0:
                continue
            id_list.append(result_item.tag_id)
            ap_pose_r_x_str = ""
            ap_pose_r_y_str = ""
            ap_pose_r_z_str = ""

            if result_item.pose_R is not None:
                for i in range(3):
                    ap_pose_r_x_str = (
                        ap_pose_r_x_str + str(round(result_item.pose_R[0][i], 4)) + " "
                    )
                    ap_pose_r_y_str = (
                        ap_pose_r_y_str + str(round(result_item.pose_R[1][i], 4)) + " "
                    )
                    ap_pose_r_z_str = (
                        ap_pose_r_z_str + str(round(result_item.pose_R[2][i], 4)) + " "
                    )

            if result_item.pose_t is not None:
                pose_t_x_list.append(result_item.pose_t[0])
                pose_t_y_list.append(result_item.pose_t[1])
                pose_t_z_list.append(result_item.pose_t[2])

            pose_r_x_list.append(ap_pose_r_x_str)
            pose_r_y_list.append(ap_pose_r_y_str)
            pose_r_z_list.append(ap_pose_r_z_str)

        self.vision_nt_id.set(id_list)
        self.vision_nt_tx.set(pose_t_x_list)
        self.vision_nt_ty.set(pose_t_y_list)
        self.vision_nt_tx.set(pose_t_z_list)
        self.vision_nt_rx.set(pose_r_x_list)
        self.vision_nt_ry.set(pose_r_y_list)
        self.vision_nt_rz.set(pose_r_z_list)

        current_time = time.time()
        analysis_et = current_time - start_time
        total_et = current_time - self.frame_time
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
                self.draw_text(image, f"X {result_item.pose_t.item(0):.2f}m", (c_x, c_y + 80))
                self.draw_text(image, f"Y {result_item.pose_t.item(1):.2f}m", (c_x, c_y + 120))
                self.draw_text(image, f"Z {result_item.pose_t.item(2):.2f}m", (c_x, c_y + 160))

    # these are white with black outline
    def draw_text(self, image, msg, loc):
        cv2.putText(image, msg, loc, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 6)
        cv2.putText(image, msg, loc, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)


def main():
    print("main")

    # full frame, 2x2, to set the detector mode
    fullwidth = 1664  # slightly larger than the detector, to match stride
    fullheight = 1232

    # lores for apriltag detector
    # option 1: big, all the pixels yields 270ms delay, but seems even higher
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

    camera = Picamera2()
    camera_config = camera.create_video_configuration(
        buffer_count=4,  # no need for extra buffers, dropping frames is fine
        main={
            "format": "YUV420",
            "size": (fullwidth, fullheight),
        },
        lores={"format": "YUV420", "size": (width, height)},
        encode="lores",
    )
    print(camera_config)
    camera.configure(camera_config)

    # Roborio IP: 10.1.0.2
    # Pi IP: 10.1.0.21

    output = TagFinder(width, height)
    camera.pre_callback = output.pre_callback
    camera.start()
    try:
        while True:
            time.sleep(1)
    finally:
        camera.stop_recording()

main()