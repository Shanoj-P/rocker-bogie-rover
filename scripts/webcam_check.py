import cv2
import numpy as np

# Open both USB webcams
left_cam = cv2.VideoCapture(2, cv2.CAP_V4L2)
right_cam = cv2.VideoCapture(4, cv2.CAP_V4L2)

# Check if opened correctly
if not left_cam.isOpened() or not right_cam.isOpened():
    print("Error: Could not open one or both cameras.")
    exit()

while True:
    retL, frameL = left_cam.read()
    retR, frameR = right_cam.read()

    if not retL or not retR:
        print("Error: Could not read frames.")
        break

    # Resize for display if needed
    frameL = cv2.resize(frameL, (640, 480))
    frameR = cv2.resize(frameR, (640, 480))

    # Combine both frames side-by-side
    stereo_view = np.hstack((frameL, frameR))
    cv2.imshow("Stereo Camera View", stereo_view)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

left_cam.release()
right_cam.release()
cv2.destroyAllWindows()
