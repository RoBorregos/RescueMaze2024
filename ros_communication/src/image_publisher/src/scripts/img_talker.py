#!/usr/bin/env python3
#create a publisher node that publishes images from the camera
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def gstreamer_pipeline(
    sensor_id=1,
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=90,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def img_talker():
    pub = rospy.Publisher('image', Image, queue_size=10)
    rospy.init_node('img_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    bridge = CvBridge()
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            msg = bridge.cv2_to_imgmsg(frame, encoding="rgb8")
            pub.publish(msg)
            rospy.loginfo("Sended image")
            rate.sleep()
    cap.release()

if __name__ == '__main__':
    try:
        img_talker()
    except rospy.ROSInterruptException:
        pass







