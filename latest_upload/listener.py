#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import threading
from surround_view import param_settings as settings
from surround_view import FisheyeCameraModel, display_image, BirdView
import os
import undistort
from PIL import Image
import numpy as np

bridge = CvBridge()

cam_front_tensor_img = None
cam_front_tensor_img_lock = threading.Lock()
cam_left_tensor_img = None
cam_left_tensor_img_lock = threading.Lock()
cam_right_tensor_img = None
cam_right_tensor_img_lock = threading.Lock()
cam_back_tensor_img = None
cam_back_tensor_img_lock = threading.Lock()

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
    global cam_back_tensor_img, cam_back_tensor_img_lock, bridge
    cam_back_tensor_img = general_callback(msg, bridge, cam_back_tensor_img_lock, tag="cam_back")

def ros_thread_function(camera_label):
    camera_tag = camera_label.lower().split("_")[1]
    topic_name = "/driver/fisheye/{}/compressed".format(camera_tag)
    callback_str = "{}_callback_function".format(camera_tag)
    rospy.Subscriber(topic_name, CompressedImage, eval(callback_str), queue_size=1)
    rospy.spin()


def image_show1():
    global cam_front_tensor_img, cam_back_tensor_img, cam_left_tensor_img, cam_right_tensor_img
    names = settings.camera_names  # 获取相机名称
    yamls = [os.path.join(os.getcwd(), "yaml", name + ".yaml") for name in names]  # 获取相机的yaml文件
    camera_models = [FisheyeCameraModel(camera_file, camera_name) for camera_file, camera_name in
                     zip(yamls, names)]  # 鱼眼镜头处理方式完成ipm处理

    while not rospy.is_shutdown():
        images = [cam_front_tensor_img, cam_back_tensor_img, cam_left_tensor_img, cam_right_tensor_img]

        projected=[]
        for image, camera, camera_name in zip(images,camera_models,names):
            if image is not None:
                image=undistort.main(camera_name, image)
                image=camera.project(image)
                image = camera.flip(image)
                projected.append(image)
                # cv2.imshow("image", projected[0])
                # cv2.waitKey(1)
        # print(len(projected))
        if len(projected)==4:
            birdview = BirdView()
            Gmat, Mmat = birdview.get_weights_and_masks(projected)  # 获取权重并求得掩膜
            birdview.update_frames(projected)  # 更新图像
            birdview.make_luminance_balance().stitch_all_parts()  # 获取拼接图像
            birdview.make_white_balance()  # 获取白平衡图像
            birdview.copy_car_image()
            cv2.imshow("BirdView Result", birdview.image)
            cv2.waitKey(1)
            # ret = display_image("BirdView Result", birdview.image)
            # if ret > 0:
            #     Image.fromarray((Gmat * 255).astype(np.uint8)).save("weights.png")
            #     Image.fromarray(Mmat.astype(np.uint8)).save("masks.png")

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
