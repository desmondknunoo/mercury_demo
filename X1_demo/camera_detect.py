import cv2
from uvc_camera import UVCCamera
import stag
import numpy as np
from pymycobot import Mercury
import time
from marker_utils import *
import os
import threading
np.set_printoptions(suppress=True, formatter={'float_kind': '{:.2f}'.format})


class camera_detect:
    #Camera parameter initialize
    def __init__(self, camera_id, marker_size, mtx, dist, arm, tool_len=174, camera_len=70):
        self.camera_id = camera_id
        self.tool_len = tool_len
        self.camera_len = camera_len
        self.mtx = mtx
        self.dist = dist
        self.marker_size = marker_size
        self.arm = arm
        self.camera = UVCCamera(self.camera_id, self.mtx, self.dist)

        # Matrix_TC = np.array([[0, -1, 0, Camera_LEN],  # 手眼矩阵
        #                       [1, 0, 0, -0],
        #                       [0, 0, 1, -Tool_LEN],
        #                       [0, 0, 0, 1]])

        # Matrix_TC = np.array([[0, 1, 0, Camera_LEN],
        #                       [-1, 0, 0, 0],
        #                       [0, 0, 1, -Tool_LEN],
        #                       [0, 0, 0, 1]])
        if arm == 0:
            self.origin_anglesL_vertical = [-83.7, -5.91, 0, -123.05, 123.27, 79.87, 63.91]  # 设置左臂观察点
            self.origin_anglesL_horizontal = [-42.88, 6.93, 0, -87.77, 85.63, 47.29, 53.57]
            if os.path.exists("left_EyesInHand_matrix.npy") and False:
                load_data = np.load("left_EyesInHand_matrix.npy")
                self.EyesInHand_matrix = load_data
            else:
                self.EyesInHand_matrix = np.array([[0, 1, 0, camera_len],
                               [-1, 0, 0, -15],
                               [0, 0, 1, -tool_len],
                               [0, 0, 0, 1]])
        else:
            self.origin_anglesL_vertical = [83.7, -5.91, 0, -123.05, -123.27, 90, -63.91]  # 设置右臂观察点
            self.origin_anglesL_horizontal = [42.88, 6.93, 0, -87.77, -85.63, 47.29, -53.57]
            if os.path.exists("right_EyesInHand_matrix.npy") and False:
                load_data = np.load("right_EyesInHand_matrix.npy")
                self.EyesInHand_matrix = load_data
            else:
                self.EyesInHand_matrix = np.array([[0, 1, 0, camera_len],
                               [-1, 0, 0, 20],
                               [0, 0, 1, -tool_len],
                               [0, 0, 0, 1]])
        self.camera_open()

        
    def camera_open(self):
        self.camera.capture()  # 打开摄像头

    def camera_open_loop(self):
        while True:
            self.camera.update_frame()  # 刷新相机界面
            frame = self.camera.color_frame()  # 获取当前帧
            cv2.imshow("img", frame)
            # cv2.waitKey(1)
            # 按下键盘任意键退出
            if cv2.waitKey(1) & 0xFF != 255:
                break

    # 读取Camera坐标（单次）
    def stag_identify(self):
        self.camera.update_frame()  # 刷新相机界面
        frame = self.camera.color_frame()  # 获取当前帧
        (corners, ids, rejected_corners) = stag.detectMarkers(frame, 11)  # 获取画面中二维码的角度和id
        marker_pos_pack = self.calc_markers_base_position(corners, ids)  # 获取物的坐标(相机系)
        print("Camera coords = ", marker_pos_pack)
        return marker_pos_pack

    # 读取Camera坐标（循环）
    def stag_identify_loop(self):
        while True:
            self.camera.update_frame()  # 刷新相机界面
            frame = self.camera.color_frame()  # 获取当前帧
            (corners, ids, rejected_corners) = stag.detectMarkers(frame, 11)  # 获取画面中二维码的角度和id
            marker_pos_pack = self.calc_markers_base_position(corners, ids)  # 获取物的坐标(相机系)
            print("Camera coords = ", marker_pos_pack)
            cv2.imshow("img", frame)
            # cv2.waitKey(1)
            # 按下键盘任意键退出
            if cv2.waitKey(1) & 0xFF != 255:
                break

    def vision_trace(self, mode, ml):
        ml.set_gripper_mode(0)
        # input("enter any key start vision trace")
        sp = 60  # 设置移动速度
        ml.set_gripper_value(0, 100)  # 闭合夹爪

        if mode == 0:   #水平面抓取
            ml.send_angles(self.origin_anglesL_horizontal, sp)  # 移动到观测点
            self.waitl(ml)  # 等待机械臂运动结束
            # input("enter any key to start trace")
            
            target_coords = self.stag_robot_identify(ml)
            print(target_coords)

            ml.set_gripper_value(100, 100)  # 打开夹爪
            time.sleep(1)
            ml.send_base_coords(target_coords, sp)  # 机械臂移动到二维码上方
            self.waitl(ml)  # 等待机械臂运动结束
            ml.send_base_coord(3, target_coords[2] - 30, sp)  # 机械臂沿z轴向下移动
            self.waitl(ml)  # 等待机械臂运动结束
            ml.set_gripper_value(20, 100)  # 闭合夹爪
            time.sleep(1)  # 等待夹爪闭合
            ml.send_base_coord(3, target_coords[2] + 30, sp)  # 抬起夹爪
            self.waitl(ml)
            ml.send_base_coord(3, target_coords[2] - 30, sp)  # 放下夹爪
            self.waitl(ml)
            ml.set_gripper_value(100, 100)  # 打开夹爪
            time.sleep(2)
            ml.send_base_coord(3, target_coords[2] + 30, sp)  # 抬起夹爪
            self.waitl(ml)
            ml.send_angles(self.origin_anglesL_horizontal, sp)  # 移动到观测点
            self.waitl(ml)  # 等待机械臂运动结束
            ml.set_gripper_value(0, 100)  # 打开夹爪
            time.sleep(2)
            
        elif mode == 1:     #竖直面抓取
            ml.send_angles(self.origin_anglesL_vertical, sp)  # 移动到观测点
            self.waitl(ml)  # 等待机械臂运动结束
            input("enter any key to start trace")
            
            target_coords = self.stag_robot_identify(ml)
            print(target_coords)

            ml.set_gripper_value(100, 100)  # 打开夹爪
            time.sleep(1)
            ml.send_base_coords(target_coords, sp)  # 机械臂移动到二维码前方
            self.waitl(ml)  # 等待机械臂运动结束
            ml.send_base_coord(1, target_coords[0] + 30, sp)  # 机械臂沿x轴向前移动
            self.waitl(ml)  # 等待机械臂运动结束
            ml.set_gripper_value(20, 100)  # 闭合夹爪
            time.sleep(1)  # 等待夹爪闭合
            ml.send_base_coord(1, target_coords[0] - 30, sp)  # 收回夹爪
            self.waitl(ml)
            ml.set_gripper_value(100, 100)  # 打开夹爪
            

    def CalculatRotationMatrix(self, camera_coords_ob, camera_coords_x, camera_coords_y, camera_coords_z):
        ob = np.array(camera_coords_ob[0:3])
        x = np.array(camera_coords_x[0:3])
        y = np.array(camera_coords_y[0:3])
        z = np.array(camera_coords_z[0:3])

        x_dis = x - ob
        y_dis = y - x
        z_dis = z - y

        print(x_dis)
        print(y_dis)
        print(z_dis)

        max_x = 0
        max_y = 0
        max_z = 0

        axis_x = 0
        axis_y = 0
        axis_z = 0

        for i in range(3):
            if abs(x_dis[i]) > max_x:
                max_x = abs(x_dis[i])
                axis_x = i

            if abs(y_dis[i]) > max_y:
                max_y = abs(y_dis[i])
                axis_y = i

            if abs(z_dis[i]) > max_z:
                max_z = abs(z_dis[i])
                axis_z = i
        print(axis_x, axis_y, axis_z)
        EysInHand = np.array([[0, 0, 0, self.camera_len],
                              [0, 0, 0, 0],
                              [0, 0, 0, -self.tool_len],
                              [0, 0, 0, 1]])
        print(x_dis[axis_x])
        print(y_dis[axis_y])
        print(z_dis[axis_z])
        EysInHand[0][axis_x] = x_dis[axis_x] / abs(x_dis[axis_x])
        EysInHand[1][axis_y] = y_dis[axis_y] / abs(y_dis[axis_y])
        EysInHand[2][axis_z] = z_dis[axis_z] / abs(z_dis[axis_z])

        print(EysInHand)
        return EysInHand
    


    def EyeInHand_calibration(self, ml):
        # move to observe points
        if self.arm == 0:
            observe_anglesL = [0, 0, -90, 90, 90, -30]
        else:
            observe_anglesL = [0, 0, -90, -90, 90, 30]
        
        input("Enter any key move to observe point")
        ml.send_angles(observe_anglesL, 5)
        # mr.send_angles(observe_anglesL, 5)

        input("Enter any key when the movement stop")

        # open camera and identify the stag code
        incre = 50
        # Left
        camera_coords_ob = self.stag_identify()  # save camera Coords
        robot_coords_ob = ml.get_base_coords()  # save robot coords
        print("camera_coords_ob", camera_coords_ob)
        print("robot_coords_ob", robot_coords_ob)

        input("Enter any key move to next x- observe point")
        ml.send_base_coord(1, robot_coords_ob[0] - incre, 10)
        input("Enter any key when the movement stop")
        camera_coords_x = self.stag_identify()  # save camera Coords
        robot_coords_x = ml.get_base_coords()  # save robot coords
        print("camera_coords_x", camera_coords_x)
        print("robot_coords_x", robot_coords_x)

        input("Enter any key move to next y+ observe point")
        ml.send_base_coord(2, robot_coords_ob[1] + incre, 10)
        input("Enter any key when the movement stop")
        camera_coords_y = self.stag_identify()  # save camera Coords
        robot_coords_y = ml.get_base_coords()  # save robot coords
        print("camera_coords_y", camera_coords_y)
        print("robot_coords_y", robot_coords_y)

        input("Enter any key move to next z+ observe point")
        ml.send_base_coord(3, robot_coords_ob[2] + incre, 10)
        input("Enter any key when the movement stop")
        camera_coords_z = self.stag_identify()  # save camera Coords
        robot_coords_z = ml.get_base_coords()  # save robot coords
        print("camera_coords_z", camera_coords_z)
        print("robot_coords_z", robot_coords_z)

        self.EyesInHand_matrix = self.CalculatRotationMatrix(camera_coords_ob, camera_coords_x, camera_coords_y, camera_coords_z)
        print(self.EyesInHand_matrix)
        # 0左臂，1右臂
        if self.arm == 0:
            np.save("left_EyesInHand_matrix", self.EyesInHand_matrix)
        else:
            np.save("right_EyesInHand_matrix", self.EyesInHand_matrix)

    # 获取物体坐标(相机系)
    def calc_markers_base_position(self, corners, ids):
        if len(corners) == 0:
            return []
        rvecs, tvecs = solve_marker_pnp(corners, self.marker_size, self.mtx, self.dist)  # 通过二维码角点获取物体旋转向量和平移向量
        for i, tvec, rvec in zip(ids, tvecs, rvecs):
            tvec = tvec.squeeze().tolist()
            rvec = rvec.squeeze().tolist()
            rotvector = np.array([[rvec[0], rvec[1], rvec[2]]])
            Rotation = cv2.Rodrigues(rotvector)[0]  # 将旋转向量转为旋转矩阵
            Euler = self.CvtRotationMatrixToEulerAngle(Rotation)  # 将旋转矩阵转为欧拉角
            target_coords = np.array([tvec[0], tvec[1], tvec[2], Euler[0], Euler[1], Euler[2]])  # 物体坐标(相机系)
        return target_coords

    def stag_robot_identify(self, ml):
        marker_pos_pack = self.stag_identify()
        target_coords = np.array(ml.get_base_coords())  # 获取机械臂当前坐标
        print("target_coords", target_coords)
        cur_coords = target_coords.copy()
        cur_coords[-3:] *= (np.pi / 180)  # 将角度值转为弧度值
        fact_bcl = self.Eyes_in_hand(cur_coords, marker_pos_pack, self.EyesInHand_matrix)  # 通过矩阵变化将物体坐标(相机系)转成(基坐标系)
        
        for i in range(3):
            target_coords[i] = fact_bcl[i]
        
        return target_coords

    # 将旋转矩阵转为欧拉角
    def CvtRotationMatrixToEulerAngle(self, pdtRotationMatrix):
        pdtEulerAngle = np.zeros(3)
        pdtEulerAngle[2] = np.arctan2(pdtRotationMatrix[1, 0], pdtRotationMatrix[0, 0])
        fCosRoll = np.cos(pdtEulerAngle[2])
        fSinRoll = np.sin(pdtEulerAngle[2])
        pdtEulerAngle[1] = np.arctan2(-pdtRotationMatrix[2, 0],
                                      (fCosRoll * pdtRotationMatrix[0, 0]) + (fSinRoll * pdtRotationMatrix[1, 0]))
        pdtEulerAngle[0] = np.arctan2((fSinRoll * pdtRotationMatrix[0, 2]) - (fCosRoll * pdtRotationMatrix[1, 2]),
                                      (-fSinRoll * pdtRotationMatrix[0, 1]) + (fCosRoll * pdtRotationMatrix[1, 1]))
        return pdtEulerAngle

    # 将欧拉角转为旋转矩阵
    def CvtEulerAngleToRotationMatrix(self, ptrEulerAngle):
        ptrSinAngle = np.sin(ptrEulerAngle)
        ptrCosAngle = np.cos(ptrEulerAngle)
        ptrRotationMatrix = np.zeros((3, 3))
        ptrRotationMatrix[0, 0] = ptrCosAngle[2] * ptrCosAngle[1]
        ptrRotationMatrix[0, 1] = ptrCosAngle[2] * ptrSinAngle[1] * ptrSinAngle[0] - ptrSinAngle[2] * ptrCosAngle[0]
        ptrRotationMatrix[0, 2] = ptrCosAngle[2] * ptrSinAngle[1] * ptrCosAngle[0] + ptrSinAngle[2] * ptrSinAngle[0]
        ptrRotationMatrix[1, 0] = ptrSinAngle[2] * ptrCosAngle[1]
        ptrRotationMatrix[1, 1] = ptrSinAngle[2] * ptrSinAngle[1] * ptrSinAngle[0] + ptrCosAngle[2] * ptrCosAngle[0]
        ptrRotationMatrix[1, 2] = ptrSinAngle[2] * ptrSinAngle[1] * ptrCosAngle[0] - ptrCosAngle[2] * ptrSinAngle[0]
        ptrRotationMatrix[2, 0] = -ptrSinAngle[1]
        ptrRotationMatrix[2, 1] = ptrCosAngle[1] * ptrSinAngle[0]
        ptrRotationMatrix[2, 2] = ptrCosAngle[1] * ptrCosAngle[0]
        return ptrRotationMatrix

    # 坐标转换为齐次变换矩阵，（x,y,z,rx,ry,rz）单位rad
    def Transformation_matrix(self,coord):
        position_robot = coord[:3]
        pose_robot = coord[3:]
        RBT = self.CvtEulerAngleToRotationMatrix(pose_robot)  # 将欧拉角转为旋转矩阵
        PBT = np.array([[position_robot[0]],
                        [position_robot[1]],
                        [position_robot[2]]])
        temp = np.concatenate((RBT, PBT), axis=1)
        array_1x4 = np.array([[0, 0, 0, 1]])
        matrix = np.concatenate((temp, array_1x4), axis=0)  # 将两个数组按行拼接起来
        return matrix

    def Eyes_in_hand(self, coord, camera, Matrix_TC):
        Position_Camera = np.transpose(camera[:3])  # 相机坐标
        Matrix_BT = self.Transformation_matrix(coord)  # 机械臂坐标矩阵

        Position_Camera = np.append(Position_Camera, 1)  # 物体坐标（相机系）
        Position_B = Matrix_BT @ Matrix_TC @ Position_Camera  # 物体坐标（基坐标系）
        return Position_B

    # 等待机械臂运行结束
    def waitl(self, ml):
        time.sleep(0.2)
        while (ml.is_moving()):
            time.sleep(0.03)



if __name__ == "__main__":
    camera_params = np.load("camera_params.npz")  # 相机配置文件
    mtx, dist = camera_params["mtx"], camera_params["dist"]
    ml = Mercury("/dev/left_arm")  # 设置左臂端口
    mr = Mercury("/dev/right_arm")  # 设置右臂端口
    ml_obj = camera_detect("/dev/left_camera", 32, mtx, dist, 0)
    # ml_obj.camera_open_loop()
    # 3 左摄像头  6右
    mr_obj = camera_detect("/dev/right_camera", 32, mtx, dist, 1)
    threading.Thread(target=mr_obj.vision_trace, args=(0,mr, )).start()
    threading.Thread(target=ml_obj.vision_trace, args=(0,ml, )).start()
    # mr_obj.vision_trace(0, mr)
    # ml_obj.vision_trace(0, ml)
