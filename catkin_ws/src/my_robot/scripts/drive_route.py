#!/usr/bin/env python3
"""Drive the README's mapping route on its own: west-room lap, east-room lap, repeated.

A stand-in for the human at the teleop keyboard when keys can't be held down (e.g. over a remote
desktop). It publishes the same /cmd_vel teleop does, steering by /odom towards each waypoint in turn.

    rosrun my_robot drive_route.py _laps:=3
"""

import math

# Waypoints in the odom frame (= World frame: the robot starts at the Start Pose, (0, 0) facing east).
# Each stays about 0.5 m clear of walls and Furniture.
WEST_LAP = [(-0.8, 0.0), (-0.9, 1.9), (-3.0, 1.9), (-3.1, 0.0), (-2.5, -0.9), (-2.3, -2.0),
            (-1.0, -2.2), (-0.8, -1.0), (-0.6, 0.0)]
EAST_LAP = [(0.8, 0.0), (0.9, -2.0), (2.6, -2.1), (2.9, -0.8), (3.0, 0.8), (2.9, 2.45),
            (1.3, 2.4), (0.8, 1.2), (0.6, 0.0)]
ROUTE = WEST_LAP + EAST_LAP

REACHED = 0.15        # m: close enough to a waypoint to move on
TURN_IN_PLACE = 0.35  # rad: heading error above which the robot stops and turns first
MAX_SPEED = 0.25      # m/s: slow, so images don't blur
MAX_TURN = 0.6        # rad/s
TURN_GAIN = 1.5


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def command(x, y, yaw, goal):
    """Return (linear, angular) velocity towards goal, or None once the goal is reached."""
    dx, dy = goal[0] - x, goal[1] - y
    distance = math.hypot(dx, dy)
    if distance < REACHED:
        return None
    error = wrap(math.atan2(dy, dx) - yaw)
    angular = max(-MAX_TURN, min(MAX_TURN, TURN_GAIN * error))
    if abs(error) > TURN_IN_PLACE:
        return 0.0, angular
    return min(MAX_SPEED, distance), angular


def main():
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from tf.transformations import euler_from_quaternion

    rospy.init_node("drive_route")
    laps = rospy.get_param("~laps", 3)
    pose = {}

    def on_odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pose["now"] = (p.x, p.y, euler_from_quaternion([q.x, q.y, q.z, q.w])[2])

    rospy.Subscriber("/odom", Odometry, on_odom)
    cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.on_shutdown(lambda: cmd_vel.publish(Twist()))
    rate = rospy.Rate(10)
    while "now" not in pose and not rospy.is_shutdown():
        rate.sleep()

    for lap in range(laps):
        rospy.loginfo("drive_route: lap %d of %d", lap + 1, laps)
        for goal in ROUTE:
            while not rospy.is_shutdown():
                velocity = command(*pose["now"], goal)
                if velocity is None:
                    break
                twist = Twist()
                twist.linear.x, twist.angular.z = velocity
                cmd_vel.publish(twist)
                rate.sleep()
    cmd_vel.publish(Twist())
    rospy.loginfo("drive_route: done")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:  # rospy.ROSInterruptException on Ctrl+C: stop quietly, the robot is already stopped
        if type(error).__name__ != "ROSInterruptException":
            raise
