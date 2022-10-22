from cscore import CameraServer
from networktables import NetworkTables

import cv2
import json
import numpy as np
import time
import draw
from pupil_apriltags import Detector
from io import BytesIO
from picamera import PiCamera

def main():
    print('main initiated')
    #input_stream = BytesIO()
    width = 1280
    height = 720
    camera = PiCamera(resolution=(width, height),
    framerate = 28,
    sensor_mode = 1)

    camera.shutter_speed = 5000
    camera.iso = 800
    camera.start_preview()
    print('camera set up')
    time.sleep(2)
    #camera.capture(input_stream, 'jpeg')

    cs = CameraServer.getInstance()


    output_stream = cs.putVideo("Processed", width, height)
    print('stream initiated')

    #Roborio IP: 10.1.0.2
    #Pi IP: 10.1.0.21
    NetworkTables.initialize(server = '10.1.0.2')
    print('NetworkTables idnitiated')
    # Table for vision output information
    vision_nt = NetworkTables.getTable("Vision")

    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(height, width, 3), dtype=np.uint8)

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
        camera.capture(img, 'bgr')
        #print('camera capture')

        gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        result = at_detector.detect(
            gray_image,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=tag_size,
        )
        #print('image converted')

        output_img = np.copy(img)
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
                if i != 2:
                    ap_pose_R_x_str = ap_pose_R_x_str + ', '
                    ap_pose_R_y_str = ap_pose_R_y_str + ', '
                    ap_pose_R_z_str = ap_pose_R_z_str + ', '
            pose_t_x_list.append(r.pose_t[0])
            pose_t_y_list.append(r.pose_t[1])
            pose_t_z_list.append(r.pose_t[2])
            pose_R_x_list.append(ap_pose_R_x_str)
            pose_R_y_list.append(ap_pose_R_y_str)
            pose_R_z_list.append(ap_pose_R_z_str)

        vision_nt.putNumberArray("id", id_list)
        vision_nt.putNumberArray("pose_t_x", pose_t_x_list)
        vision_nt.putNumberArray("pose_t_y", pose_t_y_list)
        vision_nt.putNumberArray("pose_t_z", pose_t_z_list)
        vision_nt.putStringArray("pose_R_x", pose_R_x_list)
        vision_nt.putStringArray("pose_R_y", pose_R_y_list)
        vision_nt.putStringArray("pose_R_z", pose_R_z_list)

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


main()
