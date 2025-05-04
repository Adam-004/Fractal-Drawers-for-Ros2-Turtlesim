import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
import math
import time
import colorsys
import argparse
import sys

# ---- Constants
# Base triangle fits in 11x11 turtlesim window
TRIANGLE = [(3.0, 3.0), (8.0, 3.0), (5.5, 8.0)] # warning do not change triangle points!


# ---- Geometry Helpers
# Generate 3 new points (a, b, c) that divide segment p1-p2 and create a triangle peak
def generate_points(p1, p2):
    dx = (p2[0] - p1[0]) / 3.0
    dy = (p2[1] - p1[1]) / 3.0

    a = (p1[0] + dx, p1[1] + dy)
    b = (p1[0] + 2 * dx, p1[1] + 2 * dy)

    # Angle for outward triangle point (c)
    angle = math.atan2(dy, dx) - math.pi / 3
    length = math.sqrt(dx ** 2 + dy ** 2)
    c = (a[0] + math.cos(angle) * length, a[1] + math.sin(angle) * length)

    return [a, c, b]

# Recursively build the full Koch snowflake from an initial polygon
def generate_snowflake(base_points, level):
    points = base_points[:]
    points.append(points[0])
    while level > 0:
        i = 0
        new_points = []
        while i < len(points) - 1:
            p1 = points[i]
            p2 = points[i + 1]
            a, c, b = generate_points(p1, p2)
            new_points += [p1, a, c, b]
            i += 1
        new_points.append(points[-1])
        points = new_points
        level -= 1
    return points

# ---- Turtle Controller Node
class TurtleController(Node):
    def __init__(self, speed_scale=1.0):
        super().__init__('koch_turtle')
        self.speed_scale = speed_scale
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.get_pose, 10)
        self.pose = None
        self.timer = self.create_timer(0.1, lambda: None)  # Required for spin to work

    # Get turtle's current pose
    def get_pose(self, msg):
        self.pose = msg

    # Move turtle to a point (x, y)
    def go_to_point(self, x, y):
        while self.pose is None:
            rclpy.spin_once(self)

        tolerance = 0.1  # Distance threshold to stop
        while rclpy.ok():
            dx = x - self.pose.x
            dy = y - self.pose.y

            distance = math.sqrt(dx ** 2 + dy ** 2)

            angle_to_target = math.atan2(dy, dx)
            angle_diff = angle_to_target - self.pose.theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # Normalize

            twist = Twist()
            # Rotate toward target
            if abs(angle_diff) > 0.1:
                twist.angular.z = 4.0 * angle_diff * self.speed_scale

            elif distance > tolerance:  # Move forward when facing the right direction
                twist.linear.x = 3.0 * distance * self.speed_scale

            else:
                break  # Close enough

            self.publisher.publish(twist)
            rclpy.spin_once(self)
        self.stop()

    def stop(self):
        self.publisher.publish(Twist())
        time.sleep(0.02)

    # Teleport turtle to a position (for going to initial position)
    def teleport_to(self, x, y, theta=0.0):
        client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')

        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    # Set pen color, width, and on/off state
    def set_pen(self, r=255, g=0, b=0, width=2, off=False):
        client = self.create_client(SetPen, '/turtle1/set_pen')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_pen service...')

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = int(off)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

# ---- Drawing Function
# Iterate through given path points, draw segments with HSV rainbow color gradient
def draw_and_go_through_path(turtle_controller : TurtleController, path):
    total = len(path)
    for i, point in enumerate(path):
        hue = i / total  # Progress-based hue
        r, g, b = [int(255 * c) for c in colorsys.hsv_to_rgb(hue, 0.6, 1.0)]
        turtle_controller.set_pen(r=r, g=g, b=b, width=2)

        turtle_controller.go_to_point(point[0], point[1])

# ---- Main Entry Point
def main(args=None):
    # Args handling (level of Koch snowflake)
    parser = argparse.ArgumentParser(description='Draw Koch snowflake with turtlesim.')
    parser.add_argument('--level', type=int, default=2, help='Recursion depth (e.g., 1–4)')
    level = parser.parse_args(args).level

    # Set speed based on level to compensate for complexity
    speed_scale = 1.0 + (level - 1) * 0.5

    rclpy.init()
    turtle = TurtleController(speed_scale=speed_scale)

    snowflake = generate_snowflake(TRIANGLE, level)

    # Teleport to first point with pen off (no drawing)
    start_x, start_y = TRIANGLE[0]
    turtle.set_pen(off=True)
    turtle.teleport_to(start_x, start_y, 0.0)
    turtle.set_pen(r=0, g=0, b=255, width=2, off=False)

    # Draw the full snowflake
    draw_and_go_through_path(turtle, snowflake)

    rclpy.shutdown()

# Standard Python entry point
if __name__ == '__main__':
    main(sys.argv[1:])
