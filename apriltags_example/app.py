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
from picamera.array import raw_resolution

def bytes_to_yuv(data, resolution):
        fwidth, fheight = raw_resolution(resolution)
        y_len = fwidth*fheight
        a = np.frombuffer(data, dtype=np.uint8, count=y_len)
        a = a.reshape((fheight, fwidth))
        return a[:resolution.height, :resolution.width]

class GreyYUVAnalysis(PiAnalysisOutput):

    def write(self, b):
        result = super(GreyYUVAnalysis, self).write(b)
        self.analyze(bytes_to_yuv(b, self.size or self.camera.resolution))
        return result

class MyColorAnalyzer(GreyYUVAnalysis):
    def __init__(self, camera):
        super(MyColorAnalyzer, self).__init__(camera)
        cs = CameraServer.getInstance()
        #NetworkTables Rio as server
        NetworkTables.initialize(server = '10.1.0.2')
        #NetworkTables Pi as server
        #NetworkTables.initialize()
        # Wait for NetworkTables to start
        time.sleep(0.5)
        # Table for vision output information
        self.vision_nt = NetworkTables.getTable("Vision")
        self.tag_size = 0.2
        self.at_detector = Detector(families = "tag16h5")
        time.sleep(2)
        self.output_stream = cs.putVideo("Processed", self.camera.resolution.width, self.camera.resolution.height)
        self.camera_params = [357.1, 357.1, self.camera.resolution.width / 2, self.camera.resolution.height / 2]
        

    def analyze(self, a):
        start_time = time.time()
        gray_image = a
        result = self.at_detector.detect(
            gray_image,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size = self.tag_size,
        )
        
        output_img = np.copy(gray_image)

        draw.draw_result(output_img, self.camera_params, self.tag_size, result)

        id_list = []
        pose_t_x_list = []
        pose_t_y_list = []
        pose_t_z_list = []
        pose_R_x_list = []
        pose_R_y_list = []
        pose_R_z_list = []

        for r in result:
            if r.hamming > 0:
                continue
            id_list.append(r.tag_id)
            ap_pose_R_x_str = ''
            ap_pose_R_y_str = ''
            ap_pose_R_z_str = ''

            for i in range(3):
                ap_pose_R_x_str = ap_pose_R_x_str + str(round(r.pose_R[0][i], 4)) + ' '
                ap_pose_R_y_str = ap_pose_R_y_str + str(round(r.pose_R[1][i], 4)) + ' '
                ap_pose_R_z_str = ap_pose_R_z_str + str(round(r.pose_R[2][i], 4)) + ' '
            
            pose_t_x_list.append(r.pose_t[0])
            pose_t_y_list.append(r.pose_t[1])
            pose_t_z_list.append(r.pose_t[2])

            pose_R_x_list.append(ap_pose_R_x_str)
            pose_R_y_list.append(ap_pose_R_y_str)
            pose_R_z_list.append(ap_pose_R_z_str)
        
        #self.vision_nt.putString("test", 'testest')
        self.vision_nt.putNumberArray("id", id_list)

        self.vision_nt.putNumberArray("pose_t_x", pose_t_x_list)
        self.vision_nt.putNumberArray("pose_t_y", pose_t_y_list)
        self.vision_nt.putNumberArray("pose_t_z", pose_t_z_list)
        self.vision_nt.putStringArray("pose_R_x", pose_R_x_list)
        self.vision_nt.putStringArray("pose_R_y", pose_R_y_list)
        self.vision_nt.putStringArray("pose_R_z", pose_R_z_list)

        self.output_stream.putFrame(output_img)

def main():

    width = 547
    height = 307
    camera = PiCamera(resolution=(width, height),
    framerate = 28,
    sensor_mode = 5)

#1	1920x1080	16:9	1/10 <= fps <= 30	x	 	Partial	None
#2	3280x2464	4:3	1/10 <= fps <= 15	x	x	Full	None
#3	3280x2464	4:3	1/10 <= fps <= 15	x	x	Full	None
#4	1640x1232	4:3	1/10 <= fps <= 40	x	 	Full	2x2
#5	1640x922	16:9	1/10 <= fps <= 40	x	 	Full	2x2
#6	1280x720	16:9	40 < fps <= 90	x	 	Partial	2x2
#7	640x480	4:3	40 < fps <= 90	x	 	Partial	2x2
    
    #Roborio IP: 10.1.0.2
    #Pi IP: 10.1.0.21

    with MyColorAnalyzer(camera) as analyzer:
        camera.start_recording(analyzer, 'yuv')
        try:
            while True:
                camera.wait_recording(1)
        finally:
            camera.stop_recording()
main()
