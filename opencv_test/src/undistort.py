import cv2
import numpy as np
import os


class FisheyeCamera:
    def __init__(self, para_path):
        self.para_path = para_path

    def check_undistort_info(self, camera_tag, sf=4.0):
        mapx_persp_32_path = os.path.join(self.para_path, camera_tag, f"mapx_persp_32_{camera_tag}.npy")
        mapy_persp_32_path = os.path.join(self.para_path, camera_tag, f"mapy_persp_32_{camera_tag}.npy")
        if os.path.exists(mapx_persp_32_path) and os.path.exists(mapy_persp_32_path):
            mapx_persp_32 = np.load(mapx_persp_32_path)
            mapy_persp_32 = np.load(mapy_persp_32_path)
        else:
            path_ocam = os.path.join(self.para_path, camera_tag, f"calib_results_{camera_tag}.txt")
            o = self.get_ocam_model(path_ocam)
            mapx_persp, mapy_persp = self.create_perspective_undistortion_LUT(o, sf)
            mapx_persp_32 = mapx_persp.astype('float32')
            mapy_persp_32 = mapy_persp.astype('float32')
            np.save(mapx_persp_32_path, mapx_persp_32)
            np.save(mapy_persp_32_path, mapy_persp_32)
        return mapx_persp_32, mapy_persp_32

    def get_ocam_model(self, filename):
        o = {}
        with open(filename) as f:
            lines = [l for l in f]

            l = lines[2]
            data = l.split()
            o['length_pol'] = int(data[0])
            o['pol'] = [float(d) for d in data[1:]]

            l = lines[6]
            data = l.split()
            o['length_invpol'] = int(data[0])
            o['invpol'] = [float(d) for d in data[1:]]

            l = lines[10]
            data = l.split()
            o['xc'] = float(data[0])
            o['yc'] = float(data[1])

            l = lines[14]
            data = l.split()
            o['c'] = float(data[0])
            o['d'] = float(data[1])
            o['e'] = float(data[2])

            l = lines[18]
            data = l.split()
            o['height'] = int(data[0])
            o['width'] = int(data[1])

        return o

    def create_perspective_undistortion_LUT(self, o, sf):

        mapx = np.zeros((o['height'], o['width']))
        mapy = np.zeros((o['height'], o['width']))

        Nxc = o['height'] / 2.0
        Nyc = o['width'] / 2.0
        Nz = -o['width'] / sf

        for i in range(o['height']):
            for j in range(o['width']):
                M = []
                M.append(i - Nxc)
                M.append(j - Nyc)
                M.append(Nz)
                m = self.world2cam(M, o)
                mapx[i, j] = m[1]
                mapy[i, j] = m[0]

        return mapx, mapy

    def world2cam(self, point3D, o):
        point2D = []

        norm = np.linalg.norm(point3D[:2])

        if norm != 0:
            theta = np.arctan(point3D[2] / norm)
            invnorm = 1.0 / norm
            t = theta
            rho = o['invpol'][0]
            t_i = 1.0

            for i in range(1, o['length_invpol']):
                t_i *= t
                rho += t_i * o['invpol'][i]

            x = point3D[0] * invnorm * rho
            y = point3D[1] * invnorm * rho

            point2D.append(x * o['c'] + y * o['d'] + o['xc'])
            point2D.append(x * o['e'] + y + o['yc'])
        else:
            point2D.append(o['xc'])
            point2D.append(o['yc'])

        return point2D


class FisheyeUndistort:
    def __init__(self, para_path, channel_list):
        self.fisheye_camera_obj = FisheyeCamera(para_path=para_path)
        self.max_persp = self.check_total_info(check_channel=channel_list)

    def undistort_image(self, channel_tag, cv_img):
        mapx_persp_32, mapy_persp_32 = self.max_persp[channel_tag]["x"], self.max_persp[channel_tag]["y"]
        dst_undistorted = cv2.remap(cv_img, mapx_persp_32, mapy_persp_32, cv2.INTER_LINEAR)
        return dst_undistorted

    def check_total_info(self, check_channel):
        max_persp = {}
        for channel_item in check_channel:
            max_persp[channel_item] = {}
            max_persp[channel_item]["x"], max_persp[channel_item]["y"] = self.fisheye_camera_obj.check_undistort_info(
                channel_item)
        return max_persp


def main(camara_name, image):
    para_path = os.path.join(os.path.dirname(__file__), "para")
    channel_list = ["left", "right", "front", "back"]

    fisheye_undistort = FisheyeUndistort(para_path=para_path, channel_list=channel_list)

    # Example usage:
    channel_tag = camara_name
    # input_image = cv2.imread("/home/zhe-chen/Research/surround_view/surround-view-system-noted-ChangAn/images/left.png")  # Replace with your image path
    undistorted_image = fisheye_undistort.undistort_image(channel_tag, image)

    # Save or display the undistorted image as needed
    cv2.imwrite("/home/zhe-chen/Research/surround_view/surround-view-system-noted-ChangAn/images"+camara_name+".png", undistorted_image)  # Replace with your desired output path
    return undistorted_image