import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy

class ROSVideoToOpenCV:
    def __init__(self, video_topic, frame_rate=30):
        self.bridge = CvBridge()
        self.frame_rate = frame_rate
        self.video_capture = None
        self.image_sub = rospy.Subscriber(video_topic, Image, self.video_callback)

    def video_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # If video capture is not opened or not updated, initialize it
            if self.video_capture is None or not self.video_capture.isOpened():
                height, width, _ = cv_image.shape
                self.video_capture = cv2.VideoCapture(video_topic)
                self.video_capture.set(cv2.CAP_PROP_FPS, self.frame_rate)
                self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

            # Display the video stream
            cv2.imshow("ROS Video Stream", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            print(e)
        print(self.video_capture)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('ros_video_to_opencv', anonymous=True)
    video_topic = '/camera/color/image_raw'  # Replace with your actual video topic
    ros_video_to_opencv = ROSVideoToOpenCV(video_topic)
    ros_video_to_opencv.run()
