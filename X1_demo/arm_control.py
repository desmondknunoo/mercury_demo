from pymycobot import CommandGenerator, Mercury
import typing as T

# import RPi.GPIO as GPIO
import time

def get_base_coords(arm : Mercury) -> T.Union[T.List, None]:
    coords : T.List = arm.get_base_coords() # type: ignore

    for _ in range(10):
        coords : T.List = arm.get_base_coords() # type: ignore
        if coords is not None and len(coords) != 0:
            break
        
    if coords is None or len(coords) == 0:
        return None
    return coords

def move_x_increment(arm: CommandGenerator, val: float):
    coords: T.Any = arm.get_coords()
    coords[0] += val
    arm.send_coords(coords, 10)


def move_y_increment(arm: CommandGenerator, val: float):
    coords: T.Any = arm.get_coords()
    coords[1] += val
    arm.send_coords(coords, 10)


def move_z_increment(arm: CommandGenerator, val: float):
    coords: T.Any = arm.get_coords()
    coords[2] += val
    arm.send_coords(coords, 10)


# 开启吸泵
def pump_on(arm):
    arm.set_digital_output(1, 0)
    arm.set_digital_output(2, 1)


# 关闭吸泵
def pump_off(arm):
    arm.set_digital_output(1, 1)
    arm.set_digital_output(2, 0)
    time.sleep(0.05)
    arm.set_digital_output(1, 0)
