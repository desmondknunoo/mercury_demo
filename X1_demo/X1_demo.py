import time
from pymycobot import Mercury
from uvc_camera import UVCCamera
from marker_utils import *
import stag

# 夹爪工具长度
Tool_LEN = 175
# 相机中心到法兰距离
Camera_LEN = 78
np.set_printoptions(suppress=True, formatter={'float_kind': '{:.2f}'.format})
# 相机配置文件
camera_params = np.load("camera_params.npz")
mtx, dist = camera_params["mtx"], camera_params["dist"]
# 二维码大小
MARKER_SIZE = 32
# 设置左臂端口
ml = Mercury("/dev/left_arm")


# 将旋转矩阵转为欧拉角
def CvtRotationMatrixToEulerAngle(pdtRotationMatrix):
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
def CvtEulerAngleToRotationMatrix(ptrEulerAngle):
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
def Transformation_matrix(coord):
    position_robot = coord[:3]
    pose_robot = coord[3:]
    # 将欧拉角转为旋转矩阵
    RBT = CvtEulerAngleToRotationMatrix(pose_robot)
    PBT = np.array([[position_robot[0]],
                    [position_robot[1]],
                    [position_robot[2]]])
    temp = np.concatenate((RBT, PBT), axis=1)
    array_1x4 = np.array([[0, 0, 0, 1]])
    # 将两个数组按行拼接起来
    matrix = np.concatenate((temp, array_1x4), axis=0)
    return matrix


def Eyes_in_hand_left(coord, camera):
    # 相机坐标
    Position_Camera = np.transpose(camera[:3])
    # 机械臂坐标矩阵
    Matrix_BT = Transformation_matrix(coord)
    # 手眼矩阵
    Matrix_TC = np.array([[0, -1, 0, Camera_LEN],
                          [1, 0, 0, 0],
                          [0, 0, 1, -Tool_LEN],
                          [0, 0, 0, 1]])
    # 物体坐标（相机系）
    Position_Camera = np.append(Position_Camera, 1)
    # 物体坐标（基坐标系）
    Position_B = Matrix_BT @ Matrix_TC @ Position_Camera
    return Position_B


# 等待机械臂运行结束
def waitl():
    time.sleep(0.2)
    while (ml.is_moving()):
        time.sleep(0.03)


# 获取物体坐标(相机系)
def calc_markers_base_position(corners: NDArray, ids: T.List, marker_size: int, mtx: NDArray, dist: NDArray) -> T.List:
    if len(corners) == 0:
        return []
    # 通过二维码角点获取物体旋转向量和平移向量
    rvecs, tvecs = solve_marker_pnp(corners, marker_size, mtx, dist)
    for i, tvec, rvec in zip(ids, tvecs, rvecs):
        tvec = tvec.squeeze().tolist()
        rvec = rvec.squeeze().tolist()
        rotvector = np.array([[rvec[0], rvec[1], rvec[2]]])
        # 将旋转向量转为旋转矩阵
        Rotation = cv2.Rodrigues(rotvector)[0]
        # 将旋转矩阵转为欧拉角
        Euler = CvtRotationMatrixToEulerAngle(Rotation)
        # 物体坐标(相机系)
        target_coords = np.array([tvec[0], tvec[1], tvec[2], Euler[0], Euler[1], Euler[2]])
    return target_coords


if __name__ == "__main__":
    # 设置摄像头id
    camera = UVCCamera("/dev/left_camera", mtx, dist)
    # 打开摄像头
    camera.capture()
    # 设置左臂观察点
    origin_anglesL = [-44.24, 15.56, 0.0, -102.59, 65.28, 52.06, 23.49]
    # 设置夹爪运动模式
    ml.set_gripper_mode(0)
    # 设置工具坐标系
    ml.set_tool_reference([0, 0, Tool_LEN, 0, 0, 0])
    # 将末端坐标系设置为工具
    ml.set_end_type(1)
    # 设置移动速度
    sp = 40
    # 移动到观测点
    ml.send_angles(origin_anglesL, sp)
    # 等待机械臂运动结束
    waitl()
    # 刷新相机界面
    camera.update_frame()
    # 获取当前帧
    frame = camera.color_frame()
    # 获取画面中二维码的角度和id
    (corners, ids, rejected_corners) = stag.detectMarkers(frame, 11)
    # 获取物的坐标(相机系)
    marker_pos_pack = calc_markers_base_position(corners, ids, MARKER_SIZE, mtx, dist)
    # 获取机械臂当前坐标
    cur_coords = np.array(ml.get_base_coords())
    # 将角度值转为弧度值
    cur_bcl = cur_coords.copy()
    cur_bcl[-3:] *= (np.pi / 180)
    # 通过矩阵变化将物体坐标(相机系)转成(基坐标系)
    fact_bcl = Eyes_in_hand_left(cur_bcl, marker_pos_pack)
    target_coords = cur_coords.copy()
    target_coords[0] = fact_bcl[0]
    target_coords[1] = fact_bcl[1]
    target_coords[2] = fact_bcl[2] + 50
    # 机械臂移动到二维码上方
    ml.send_base_coords(target_coords, 30)
    # 等待机械臂运动结束
    waitl()
    # 打开夹爪
    ml.set_gripper_value(100, 100)
    # 机械臂沿z轴向下移动
    ml.send_base_coord(3, fact_bcl[2], 10)
    # 等待机械臂运动结束
    waitl()
    # 闭合夹爪
    ml.set_gripper_value(20, 100)
    # 等待夹爪闭合
    time.sleep(2)
    # 抬起夹爪
    ml.send_base_coord(3, fact_bcl[2] + 50, 10)