#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import threading

bridge = CvBridge()

cam_front_tensor_img = None
cam_front_tensor_img_lock = threading.Lock()
cam_left_tensor_img = None
cam_left_tensor_img_lock = threading.Lock()
cam_right_tensor_img = None
cam_right_tensor_img_lock = threading.Lock()
cam_rear_tensor_img = None
cam_rear_tensor_img_lock = threading.Lock()

def general_callback(msg, bridge, img_lock, tag="no"):
    cv_img = bridge.compressed_imgmsg_to_cv2(msg)
    img_lock.acquire()
    img_lock.release()
    return cv_img

def front_callback_function(msg):
    global cam_front_tensor_img, cam_front_tensor_img_lock, bridge
    cam_front_tensor_img = general_callback(msg, bridge, cam_front_tensor_img_lock, tag="cam_front")

def left_callback_function(msg):
    global cam_left_tensor_img, cam_left_tensor_img_lock, bridge
    cam_left_tensor_img = general_callback(msg, bridge, cam_left_tensor_img_lock, tag="cam_left")

def right_callback_function(msg):
    global cam_right_tensor_img, cam_right_tensor_img_lock, bridge
    cam_right_tensor_img = general_callback(msg, bridge, cam_right_tensor_img_lock, tag="cam_right")

def back_callback_function(msg):
    global cam_rear_tensor_img, cam_rear_tensor_img_lock, bridge
    cam_rear_tensor_img = general_callback(msg, bridge, cam_rear_tensor_img_lock, tag="cam_back")

def ros_thread_function(camera_label):
    camera_tag = camera_label.lower().split("_")[1]
    topic_name = "/driver/fisheye/{}/compressed".format(camera_tag)
    callback_str = "{}_callback_function".format(camera_tag)
    rospy.Subscriber(topic_name, CompressedImage, eval(callback_str), queue_size=1)
    rospy.spin()

def image_show1():
    global cam_front_tensor_img
    while not rospy.is_shutdown():
        if cam_front_tensor_img is not None:
            cv2.imshow("image", cam_front_tensor_img)
            cv2.waitKey(1)
def main():
    rospy.init_node('listener', anonymous=True)
    threads = []
    for camera_label in ["CAM_front", "CAM_left", "CAM_right", "CAM_back"]:
        image_thread = threading.Thread(target=ros_thread_function, args=(camera_label,))
        image_thread.start()
        threads.append(image_thread)

    processing_thread = threading.Thread(target=image_show1)
    processing_thread.start()
    threads.append(processing_thread)


    for thread in threads:
        thread.join()

if __name__ == "__main__":
    main()
