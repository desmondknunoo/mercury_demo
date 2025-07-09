from uvc_camera import UVCCamera
from arm_control import *
from marker_utils import *
import stag
from mercury_ros_api import MapNavigation

# Gripper tool length
Tool_LEN = 175
# Distance from camera center to flange                                          
Camera_LEN = 78
np.set_printoptions(suppress=True, formatter={'float_kind': '{:.2f}'.format})
# Camera configuration file
camera_params = np.load("src/camera_params.npz")
mtx, dist = camera_params["mtx"], camera_params["dist"]
# QR code size
MARKER_SIZE = 32
# Set left arm port                                     
ml = Mercury("/dev/left_arm")

# Convert rotation matrix to Euler angles
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

# Convert Euler angles to rotation matrix
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

# Convert coordinates to homogeneous transformation matrix (x, y, z, rx, ry, rz) in radians
def Transformation_matrix(coord):
    position_robot = coord[:3]
    pose_robot = coord[3:]
    # Convert Euler angles to rotation matrix
    RBT = CvtEulerAngleToRotationMatrix(pose_robot)
    PBT = np.array([[position_robot[0]],
                    [position_robot[1]],
                    [position_robot[2]]])
    temp = np.concatenate((RBT, PBT), axis=1)
    array_1x4 = np.array([[0, 0, 0, 1]])
    # Concatenate the two arrays row-wise
    matrix = np.concatenate((temp, array_1x4), axis=0)
    return matrix

def Eyes_in_hand_left(coord, camera):
    # Camera coordinates
    Position_Camera = np.transpose(camera[:3])
    # Robotic arm coordinate matrix             
    Matrix_BT = Transformation_matrix(coord)
    # Hand-eye matrix           
    Matrix_TC = np.array([[0, -1, 0, Camera_LEN],
                          [1, 0, 0, 0],
                          [0, 0, 1, -Tool_LEN],
                          [0, 0, 0, 1]])
    # Object coordinates (camera system)
    Position_Camera = np.append(Position_Camera, 1)
    # Object coordinates (base coordinate system)         
    Position_B = Matrix_BT @ Matrix_TC @ Position_Camera
    return Position_B

# Wait for the robotic arm to finish moving
def waitl():
    time.sleep(0.2)
    while (ml.is_moving()):
        time.sleep(0.03)

# Get object coordinates (camera system)
def calc_markers_base_position(corners: NDArray, ids: T.List, marker_size: int, mtx: NDArray, dist: NDArray) -> T.List:
    if len(corners) == 0:
        return []
    # Get object rotation and translation vectors through QR code corners
    rvecs, tvecs = solve_marker_pnp(corners, marker_size, mtx, dist)
    for i, tvec, rvec in zip(ids, tvecs, rvecs):
        tvec = tvec.squeeze().tolist()
        rvec = rvec.squeeze().tolist()
        rotvector = np.array([[rvec[0], rvec[1], rvec[2]]])
        # Convert rotation vector to rotation matrix
        Rotation = cv2.Rodrigues(rotvector)[0]
        # Convert rotation matrix to Euler angles                   
        Euler = CvtRotationMatrixToEulerAngle(Rotation)
        # Object coordinates (camera system)                     
        target_coords = np.array([tvec[0], tvec[1], tvec[2], Euler[0], Euler[1], Euler[2]])
    return target_coords

if __name__ == "__main__":
     # Navigate to the target point, this point needs to be mapped and set by yourself
     map_navigation = MapNavigation()
     flag_feed_goalReached = MapNavigation.moveToGoal(1.8811798181533813, 1.25142673254013062, 0.9141818042023212,0.4053043657122249)
     # Check if the target point is reached
     if flag_feed_goalReached:
        # Set camera ID                  
        camera = UVCCamera(5, mtx, dist)
        # Open the camera
        camera.capture()
        # Set left arm observation point
        origin_anglesL = [-44.24, 15.56, 0.0, -102.59, 65.28, 52.06, 23.49]
        # Set gripper movement mode
        ml.set_gripper_mode(0)
        # Set tool coordinate system
        ml.set_tool_reference([0, 0, Tool_LEN, 0, 0, 0])
        # Set the end coordinate system as the tool        
        ml.set_end_type(1)
        # Set movement speed                 
        sp = 40
        # Move to the observation point                  
        ml.send_angles(origin_anglesL, sp)
        # Wait for the robotic arm to finish moving    
        waitl()
        # Refresh the camera interface  
        camera.update_frame()
        # Get the current frame
        frame = camera.color_frame()
        # Get the angles and IDs of the QR codes in the frame
        (corners, ids, rejected_corners) = stag.detectMarkers(frame, 11)
        # Get the object's coordinates (camera system)
        marker_pos_pack = calc_markers_base_position(corners, ids, MARKER_SIZE, mtx, dist)
        # Get the current coordinates of the robotic arm
        cur_coords = np.array(ml.get_base_coords())
        # Convert angle values to radians       
        cur_bcl = cur_coords.copy()
        cur_bcl[-3:] *= (np.pi / 180)
        # Convert the object's coordinates (camera system) to (base coordinate system) through matrix transformation
        fact_bcl = Eyes_in_hand_left(cur_bcl, marker_pos_pack)
        target_coords = cur_coords.copy()
        target_coords[0] = fact_bcl[0]
        target_coords[1] = fact_bcl[1]
        target_coords[2] = fact_bcl[2] + 50
        # Move the robotic arm above the QR code
        ml.send_base_coords(target_coords, 30)
        # Wait for the robotic arm to finish moving
        waitl()
        # Open the gripper              
        ml.set_gripper_value(100, 100)
        # Move the robotic arm down along the z-axis
        ml.send_base_coord(3, fact_bcl[2], 10)
        # Wait for the robotic arm to finish moving
        waitl()
        # Close the gripper                                                        
        ml.set_gripper_value(20, 100)

        # Wait for the gripper to close       
        time.sleep(2)

        # Lift the gripper
        ml.send_base_coord(3, fact_bcl[2] + 50, 10)
