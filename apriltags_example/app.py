from cscore import CameraServer
from networktables import NetworkTables

import cv2
import json
import numpy as np
import time
import draw
from pupil_apriltags import Detector
import io
from picamera import PiCamera
import warnings
from picamera.array import PiAnalysisOutput

def bytes_to_yuv(data):
        """
        Converts a bytes object containing YUV data to a `numpy`_ array.
        """
        a = np.frombuffer(data, dtype=np.uint8)

class GreyYUVAnalysis(PiAnalysisOutput):

    def write(self, b):
        result = super(GreyYUVAnalysis, self).write(b)
        self.analyze(bytes_to_yuv(b, self.size or self.camera.resolution))
        return result

class MyColorAnalyzer(GreyYUVAnalysis):
    def __init__(self, camera):
        super(MyColorAnalyzer, self).__init__(camera)
        self.last_color = ''
        NetworkTables.initialize(server = '10.1.0.2')
        print('NetworkTables idnitiated')
        # Table for vision output information
        self.vision_nt = NetworkTables.getTable("Vision")
        self.at_detector = Detector()
        

    def analyze(self, a):
        # Convert the average color of the pixels in the middle box
        
        gray_image = img
        result = self.at_detector.detect(
            gray_image,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=tag_size,
        )
        #print('image converted')

        output_img = np.copy(gray_image)
        draw.draw_result(output_img, camera_params, tag_size, result)
        #print('result draw')

        id_list = []
        pose_t_x_list = []
        pose_t_y_list = []
        pose_t_z_list = []
        pose_R_x_list = []
        pose_R_y_list = []
        pose_R_z_list = []

        for r in result:
            id_list.append(r.tag_id)

            ap_pose_R_x_str = ''
            ap_pose_R_y_str = ''
            ap_pose_R_z_str = ''

            for i in range(3):
                ap_pose_R_x_str = ap_pose_R_x_str + str(round(r.pose_R[i][0], 4))
                ap_pose_R_y_str = ap_pose_R_y_str + str(round(r.pose_R[i][1], 4))
                ap_pose_R_z_str = ap_pose_R_z_str + str(round(r.pose_R[i][2], 4))
            
            pose_t_x_list.append(r.pose_t[0])
            pose_t_y_list.append(r.pose_t[1])
            pose_t_z_list.append(r.pose_t[2])

            pose_R_x_list.append(ap_pose_R_x_str)
            pose_R_y_list.append(ap_pose_R_y_str)
            pose_R_z_list.append(ap_pose_R_z_str)

        self.vision_nt.putNumberArray("id", id_list)

        self.vision_nt.putNumberArray("pose_t_x", pose_t_x_list)
        self.vision_nt.putNumberArray("pose_t_y", pose_t_y_list)
        self.vision_nt.putNumberArray("pose_t_z", pose_t_z_list)

        self.vision_nt.putStringArray("pose_R_x", pose_R_x_list)
        self.vision_nt.putStringArray("pose_R_y", pose_R_y_list)
        self.vision_nt.putStringArray("pose_R_z", pose_R_z_list)

        processing_time = time.time() - start_time
        fps = 1 / processing_time
        cv2.putText(
            output_img,
            f"fps: {str(round(fps, 1))}",
            (0, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
        )
        output_stream.putFrame(output_img)
        #print('loop finish')

def main():
    print('main initiated')
    #input_stream = BytesIO()
#1	1920x1080	16:9	1/10 <= fps <= 30	x	 	Partial	None
#2	3280x2464	4:3	1/10 <= fps <= 15	x	x	Full	None
#3	3280x2464	4:3	1/10 <= fps <= 15	x	x	Full	None
#4	1640x1232	4:3	1/10 <= fps <= 40	x	 	Full	2x2
#5	1640x922	16:9	1/10 <= fps <= 40	x	 	Full	2x2
#6	1280x720	16:9	40 < fps <= 90	x	 	Partial	2x2
#7	640x480	4:3	40 < fps <= 90	x	 	Partial	2x2
    width = 1280
    height = 704
    camera = PiCamera(resolution=(width, height),
    framerate = 28,
    sensor_mode = 1)

    camera.shutter_speed = 1000
    camera.iso = 800
    camera.start_preview()
    print('camera set up')
    time.sleep(2)
    #camera.capture(input_stream, 'jpeg')



    output_stream = cs.putVideo("Processed", width, height)
    print('stream initiated')

    #Roborio IP: 10.1.0.2
    #Pi IP: 10.1.0.21

    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(height, width), dtype=np.uint8)

    # Wait for NetworkTables to start
    time.sleep(0.5)

    camera_params = [300, 300, width / 2, height / 2]
    tag_size = 0.04
    at_detector = Detector()
    print('last pre-loop')
    while True:
        start_time = time.time()
        #print(start_time)
        #input_img = input_stream.read()

        # notify output of error and skip iteration
        #if input_stream.tell() == 0:
            #output_stream.notifyError(input_stream.getError())
            #continue
        try:
            camera.capture(img, 'yuv', use_video_port = True)
        except IOError:
            pass
        #print('camera capture')

main()
