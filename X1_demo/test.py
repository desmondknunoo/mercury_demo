    from pymycobot import *
    import numpy as np
    import time

    mr = Mercury("/dev/right_arm")  #connect to Mercury right arm
    ml = Mercury("/dev/left_Arm")  #connect to Mercury left arm

    ml.over_limit_return_zero()
    mr.over_limit_return_zero()
    time.sleep(10)

    ml.set_gripper_mode(0)
    mr.set_gripper_mode(0)
    time.sleep(1)
    ml.set_gripper_value(100,10)
    mr.set_gripper_value(100,10)
    time.sleep(3)
    ml.set_gripper_value(0,10)
    mr.set_gripper_value(0,10)
    time.sleep(3)

    ml.send_angles([45, 20, -45, 45, 120, 45], 10)
    mr.send_angles([-45, 20, 45, 45, 120, -45], 10)
    time.sleep(5)
    la = ml.get_angles()
    ra = mr.get_angles()
    print("left", la)
    print("right", ra)

    ml.set_gripper_value(100,10)
    mr.set_gripper_value(100,10)
    time.sleep(3)
    ml.set_gripper_value(0,10)
    mr.set_gripper_value(0,10)
    time.sleep(3)
