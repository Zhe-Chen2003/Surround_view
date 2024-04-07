#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import threading
from PyQt5.QtWidgets import QApplication  # 导入Qt的QApplication模块

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
            cv2.imshow("image1", cam_front_tensor_img)  # 使用不同的窗口名称
            cv2.waitKey(1)

def image_show2():
    global cam_left_tensor_img
    while not rospy.is_shutdown():
        if cam_left_tensor_img is not None:
            cv2.imshow("image2", cam_left_tensor_img)  # 使用不同的窗口名称
            cv2.waitKey(1)

def main():
    # 创建Qt的QApplication对象，并在主线程中进行
    app = QApplication([])

    rospy.init_node('listener', anonymous=True)
    threads = []
    for camera_label in ["CAM_front", "CAM_left", "CAM_right", "CAM_back"]:
        image_thread = threading.Thread(target=ros_thread_function, args=(camera_label,))
        image_thread.start()
        threads.append(image_thread)

    processing_thread1 = threading.Thread(target=image_show1)
    processing_thread1.start()
    threads.append(processing_thread1)

    processing_thread2 = threading.Thread(target=image_show2)
    processing_thread2.start()
    threads.append(processing_thread2)

    for thread in threads:
        thread.join()

    # 程序结束后退出Qt应用程序
    app.quit()

if __name__ == "__main__":
    main()
