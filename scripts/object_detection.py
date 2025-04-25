#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class CubeDetector:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('cube_detector', anonymous=True)
        rospy.loginfo('started')
        
        # Create a CvBridge object
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber("/rover2/camera1/image_raw", Image, self.callback)

        # Create a VideoWriter object to save the output video (optional)
        self.output_video = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'XVID'), 20.0, (640, 480))

    def callback(self, data):
        try:
            # Convert the ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Call the cube detection function
            self.detect_cubes(cv_image)

        except Exception as e:
            rospy.logerr(e)

    def detect_cubes(self, image):
        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the color range for cube detection (example for blue cubes)
        lower_color = np.array([100, 150, 0])  # Adjust these values based on your cube color
        upper_color = np.array([140, 255, 255])

        # Create a mask for the specific color
        mask = cv2.inRange(hsv_image, lower_color, upper_color)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Draw rectangles around detected cubes
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:  # Filter out small contours
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw a green rectangle

        # Show the processed image
        cv2.imshow("Cube Detection", image)
        self.output_video.write(image)  # Write the frame to the output video

        # Exit if the user presses 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cleanup()

    def cleanup(self):
        # Release the VideoWriter and destroy all windows
        self.output_video.release()
        cv2.destroyAllWindows()
        rospy.signal_shutdown("Shutting down the cube detector node.")

if __name__ == '__main__':
    try:
        detector = CubeDetector()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        detector.cleanup()
