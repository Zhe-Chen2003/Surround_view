import os
import cv2


camera_names = ["front", "back", "left", "right"]

# --------------------------------------------------------------------
# (shift_width, shift_height): 这两个参数决定了在鸟瞰图中向标定板的外侧看多远。这两个值越大，鸟瞰图看的范围就越大，相应地远处的物体被投影后的形变也越严重，所以应酌情选择。
shift_w = 150#横向的偏移量
shift_h = 200#纵向的偏移量

# 标定板内侧边缘与车辆左右两侧的距离，标定板内侧边缘与车辆前后方的距离。cm
inn_shift_w = 20
inn_shift_h = 25

# 缝合图像的总宽度/高度
total_w = 506 + 2 * shift_w
total_h = 741 + 2 * shift_h

# 汽车左上(x_left, y_top)，右下(x_right, y_bottom)所占据的矩形区域的四个角
xl = shift_w + 140 + inn_shift_w#shift宽度+棋盘宽度+棋盘内侧边缘宽度
xr = total_w - xl
yt = shift_h + 120 + inn_shift_h
yb = yt + 451
# --------------------------------------------------------------------

project_shapes = {
    "front": (total_w, yt),
    "back":  (total_w, yt),
    "left":  (total_h, xl),
    "right": (total_h, xl)
}#投影图的大小

# 要选择的四个点的像素位置。 在运行get_projection_map.py脚本时，必须以相同的顺序单击这些像素
project_keypoints = {
    "front": [(shift_w + 160, shift_h),
              (shift_w + 340, shift_h),
              (shift_w + 160, shift_h + 100),
              (shift_w + 340, shift_h + 100)],

    "back":  [(shift_w + 180, shift_h+20 ),
              (shift_w + 320, shift_h+20 ),
              (shift_w + 180, shift_h + 120),
              (shift_w + 320, shift_h + 120)],

    "left":  [(shift_h + 306, shift_w+40),
              (shift_h + 526, shift_w+40),
              (shift_h + 306, shift_w + 140),
              (shift_h + 527, shift_w + 140)],

    "right": [(shift_h + 220, shift_w+40),
              (shift_h + 440, shift_w+40),
              (shift_h + 220, shift_w + 120),
              (shift_h + 440, shift_w + 120)]
}

car_image = cv2.imread(os.path.join(os.getcwd(), "images", "car.png"))#汽车图片
car_image = cv2.resize(car_image, (xr - xl, yb - yt))#汽车图片的大小
