import cv2
from pupil_apriltags import Detector
import draw

cam = cv2.VideoCapture(0)
if cam.read() == False:
    cam.open()
if not cam.isOpened():
    raise Exception("Cannot open camera")

width = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
camera_params = [300, 300, width / 2, height / 2]
tag_size = 0.04
at_detector = Detector()

while True:
    _, image = cam.read()
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    result = at_detector.detect(
        gray_image,
        estimate_tag_pose=True,
        camera_params=camera_params,
        tag_size=tag_size,
    )
    draw.draw_result(image, camera_params, tag_size, result)
    cv2.imshow("webcam", image)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cam.release()
cv2.destroyAllWindows()
