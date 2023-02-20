import cv2
from pupil_apriltags import Detector
import draw

image = cv2.imread("example.png")

height = image.shape[0]
width = image.shape[1]
camera_params = [500, 500, width / 2, height / 2]
tag_size = 0.02
at_detector = Detector()

gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
result = at_detector.detect(
    gray_image, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size
)

draw.draw_result(image, camera_params, tag_size, result)
cv2.imshow("Image", image)
cv2.waitKey(0)
