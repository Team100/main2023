# pylint: disable=missing-module-docstring
# pylint: disable=missing-function-docstring
# pylint: disable=missing-class-docstring

import io
import json
import time

from cscore import CameraServer

# from networktables import NetworkTables
from ntcore import NetworkTableInstance

import cv2
import numpy as np

from pupil_apriltags import Detector

# old
# from picamera import PiCamera
# new
from picamera2 import Picamera2
import libcamera

# import warnings
# old
# from picamera.array import PiAnalysisOutput
# from picamera2.array import raw_resolution

# new
from picamera2.outputs import Output
from picamera2.encoders import Encoder

def bytes_to_yuv(data, width, height):
    y_len = width * height
    array = np.frombuffer(data, dtype=np.uint8, count=y_len)
    array = array.reshape((height, width))
#    array = np.frombuffer(data, dtype=np.uint8)
#    array = array.reshape((int(height*3/2), -1))
#    array = array[: height, ]
#    print(array.shape) # (width,height) = (616,832)
#    return array
    return array[: height, : width]


# old
# class GreyYUVAnalysis(PiAnalysisOutput):
# new
class MyColorAnalyzer(Output):
    # old
    #    def write(self, b):
    # new
    def outputframe(self, buffer, keyframe, timestamp):
        # old
        #        result = super(GreyYUVAnalysis, self).write(b)
        # new
        result = len(buffer)
############
        self.analyze(bytes_to_yuv(buffer, self.width, self.height))
        #self.analyze_null(buffer, self.width, self.height)
############
        return result

    # old
    # class MyColorAnalyzer(GreyYUVAnalysis):
    def __init__(self, width, height):
        self.frame_time = time.time()
        self.width = width
        self.height = height
        #super(MyColorAnalyzer, self).__init__(camera)
# maybe the "instance' is gone?
#        camera_server = CameraServer.getInstance()
        # NetworkTables Rio as server
        # NetworkTables.initialize(server="10.1.0.14")
        # NetworkTables Pi as server (for testing)
        # NetworkTables.initialize()
        # Wait for NetworkTables to start
        #        time.sleep(0.5)
        # Table for vision output information
        # new
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
        # old
        # self.vision_nt = NetworkTables.getTable("Vision")
        self.tag_size = 0.2
        self.circle_tag_size = 0.4
        self.at_detector = Detector(families="tag16h5")
        self.at_circle_detector = Detector(families="tagCircle21h7")
        #        time.sleep(2)
#        self.output_stream = camera_server.putVideo(
        self.output_stream = CameraServer.putVideo(
            "Processed", width, height
        )
        self.camera_params = [
            357.1,
            357.1,
            width / 2,
            height / 2,
        ]

    def analyze_null(self, buffer, width, height):
        y_len = width * height # size of Y frame
# ignore chrominance
        array = np.frombuffer(buffer, dtype=np.uint8, count=y_len)
        array = array.reshape((height, width))

#        array = np.frombuffer(buffer, dtype=np.uint8)
#        array = array.reshape((int(height*3/2), -1)) # 1848x1664

#        array = cv2.cvtColor(array, cv2.COLOR_YUV2GRAY_420)
#        array = array.reshape((-1, width))
#        #array = array.reshape((height, -1))
#        #array = array.reshape((height, width, 3))
        array = array[: height, : width]
        #array = array[: height, ]
        #print(array.shape) # (width,height) = (616,832)
        self.output_stream.putFrame(array)

    def analyze(self, array):
        start_time = time.time()
        gray_image = array
 
        # turn off analysis for the moment
#        result = []

        result = self.at_detector.detect(
            gray_image,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size,
        )

        circle_result = self.at_circle_detector.detect(
            gray_image,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.circle_tag_size,
        )

        result = result + circle_result

        #output_img = np.copy(gray_image)
        output_img = gray_image

        draw_result(output_img, result)

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

        # self.vision_nt.putString("test", 'testest')
        self.vision_nt_id.set(id_list)

        self.vision_nt_tx.set(pose_t_x_list)
        self.vision_nt_ty.set(pose_t_y_list)
        self.vision_nt_tx.set(pose_t_z_list)
        self.vision_nt_rx.set(pose_r_x_list)
        self.vision_nt_ry.set(pose_r_y_list)
        self.vision_nt_rz.set(pose_r_z_list)

        current_time = time.time()
        analysis_et = (current_time - start_time)
        total_et = (current_time - self.frame_time)
        fps = 1/total_et
        self.frame_time = current_time
        draw_text(output_img, f"analysis ET(ms) {1000*analysis_et:.0f}", (5, 25))
        draw_text(output_img, f"total ET(ms) {1000*total_et:.0f}", (5, 65))
        draw_text(output_img, f"fps {fps:.1f}", (5, 105))
        self.output_stream.putFrame(output_img)


def main():
    print("main")

    # full frame, 2x2, to set the detector mode
    # would result in too much delay if we used all the pixels
    fullwidth=1664 # extra to match stride
    fullheight=1232

    # lores for apriltag detector
    # option 1: using all the pixels yields 270ms delay
    # width=1664
    # height=1232
    # option 2: medium-resolution, two circles, three squares, ~80ms
    # remember to note this delay in the kalman filter input
    # TODO: report the actual delay in network tables
    width=832
    height=616
    # option 3: very tiny, trade speed for detection distance; two circles, three squraes, ~40ms
    # width=448
    # height=308

    # old
    #    camera = PiCamera2(resolution=(width, height), framerate=28, sensor_mode=5)
    camera = Picamera2()
    # new
    camera_config = camera.create_video_configuration(
        buffer_count=2, # no need for extra buffers, dropping frames is fine
        main={"format": "YUV420", "size": (fullwidth, fullheight)}, # specify full size to eliminate crop
        lores={"format": "YUV420", "size": (width, height)},
        #raw={'format': 'SRGGB8', 'size':(width,height)},
        encode='lores'
        #{"format": "RGB888", "size": (width, height)}, raw=camera.sensor_modes[5]
    )
    print(camera_config)
    camera.configure(camera_config)
    #camera.set_controls({"ScalerCrop": [0,0,3280,2464]})

    # Roborio IP: 10.1.0.2
    # Pi IP: 10.1.0.21

    encoder = Encoder() # no-op
    output = MyColorAnalyzer(width, height)
    #camera.start_recording(analyzer, "yuv")
    camera.start_recording(encoder, output)
    try:
        while True:
            # old
            # camera.wait_recording(1)
            # new
            time.sleep(1)
    finally:
        camera.stop_recording()


# these are white with black outline
def draw_text(image, msg, loc):
    cv2.putText(image, msg, loc, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 6)
    cv2.putText(image, msg, loc, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)


def draw_result(image, result):
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
        draw_text(image, f"id {tag_id}", (c_x, c_y))
        tag_family = result_item.tag_family.decode("utf-8")
        draw_text(image, f"id {tag_family}", (c_x, c_y + 40))

        # put the translation in the image
        # the use of 'item' here is to force a scalar to format
        if result_item.pose_t is not None:
            draw_text(image, f"X {result_item.pose_t.item(0):.2f}m", (c_x, c_y + 80))
            draw_text(image, f"Y {result_item.pose_t.item(1):.2f}m", (c_x, c_y + 120))
            draw_text(image, f"Z {result_item.pose_t.item(2):.2f}m", (c_x, c_y + 160))


main()
