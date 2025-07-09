import numpy as np
from pymycobot import Mercury
# Import the vision recognition stag package
from camera_detect import camera_detect

if __name__ == "__main__":
    # Navigate to the target point, this point needs to be mapped and set by yourself
    map_navigation = MapNavigation()
    flag_feed_goalReached = MapNavigation.moveToGoal(1.8811798181533813, 1.25142673254013062, 0.9141818042023212,0.4053043657122249)
    # Check if the target point is reached
    if flag_feed_goalReached:
        camera_params = np.load("camera_params.npz")  # Camera configuration file
        mtx, dist = camera_params["mtx"], camera_params["dist"]
        ml = Mercury("/dev/left_arm")  # Set left arm port
        mr = Mercury("/dev/right_arm")  # Set right arm port
        arm = 0         # 0: Left arm, 1: Right arm
        catch_mode = 0  # 0: Horizontal grasping, 1: Vertical grasping
        # Camera ID needs to be changed according to the actual situation
        left_camera_id = 3
        right_camera_id = 6
        stag_size = 32
        # Create vision recognition objects for left and right arms
        ml_obj = camera_detect(left_camera_id, stag_size, mtx, dist, arm)
        mr_obj = camera_detect(right_camera_id, stag_size, mtx, dist, arm)
        # Move the left and right arms to the horizontal observation position for grasping
        mr_obj.vision_trace(catch_mode, mr)
        ml_obj.vision_trace(catch_mode, ml)
