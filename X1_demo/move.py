from mercury_ros_api import MapNavigation

if __name__ == "__main__":
    # Navigate to the target point, this point needs to be mapped and set by yourself
    map_navigation = MapNavigation()
    flag_feed_goalReached = map_navigation.moveToGoal(1.8811798181533813, 1.25142673254013062, 0.9141818042023212,0.4053043657122249)
    # Move the chassis forward
    map_navigation.goStraight(0.25, 5)
    # Move the chassis backward
    map_navigation.goBack(0.25, 5)
    # Move the chassis to the left
    map_navigation.turnLeft(0.25, 5)
    # Move the chassis to the right
    map_navigation.turnRight(0.25, 5)
