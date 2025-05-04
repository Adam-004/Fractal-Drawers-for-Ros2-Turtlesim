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

# ---- Geometry Helper ----

# Midpoint between two points
def midpoint(p1, p2):
    return ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)

# ---- Turtle Controller ----

class TurtleController(Node):
    def __init__(self, speed_scale=1.0):
        super().__init__('sierpinski_turtle')
        self.speed_scale = speed_scale
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pose = None
        self.timer = self.create_timer(0.1, lambda: None)

    def pose_callback(self, msg):
        self.pose = msg

    def go_to_point(self, x, y):
        while self.pose is None:
            rclpy.spin_once(self)
        tolerance = 0.01
        while rclpy.ok():
            dx = x - self.pose.x
            dy = y - self.pose.y
            distance = math.sqrt(dx ** 2 + dy ** 2)
            angle_to_target = math.atan2(dy, dx)
            angle_diff = angle_to_target - self.pose.theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            twist = Twist()
            if abs(angle_diff) > tolerance:
                twist.angular.z = 4.0 * angle_diff * self.speed_scale
            elif distance > tolerance:
                twist.linear.x = 3.0 * distance * self.speed_scale
            else:
                break
            self.publisher.publish(twist)
            rclpy.spin_once(self)
        self.stop()

    def stop(self):
        self.publisher.publish(Twist())
        time.sleep(0.02)

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

# ---- Drawing Recursive Sierpiński ----

def draw_triangle(turtle, p1, p2, p3, color_index, color_total):
    # Set color based on progress
    hue = color_index / color_total
    r, g, b = [int(255 * c) for c in colorsys.hsv_to_rgb(hue, 0.6, 1.0)]
    turtle.set_pen(r=r, g=g, b=b, width=2)

    # Move to first point without drawing
    turtle.set_pen(off=True)
    turtle.teleport_to(p1[0], p1[1], 0.0)
    turtle.set_pen(r=r, g=g, b=b, width=2)

    # Draw the triangle
    turtle.go_to_point(p2[0], p2[1])
    turtle.go_to_point(p3[0], p3[1])
    turtle.go_to_point(p1[0], p1[1])

def sierpinski(turtle, p1, p2, p3, level, current_depth, max_depth, color_index, color_total):
    if level == 0:
        draw_triangle(turtle, p1, p2, p3, color_index, color_total)
        return color_index + 1

    # Get midpoints
    m1 = midpoint(p1, p2)
    m2 = midpoint(p2, p3)
    m3 = midpoint(p3, p1)

    # Recurse into 3 new triangles
    color_index = sierpinski(turtle, p1, m1, m3, level - 1, current_depth + 1, max_depth, color_index, color_total)
    color_index = sierpinski(turtle, m1, p2, m2, level - 1, current_depth + 1, max_depth, color_index, color_total)
    color_index = sierpinski(turtle, m3, m2, p3, level - 1, current_depth + 1, max_depth, color_index, color_total)

    return color_index

# ---- Entry Point ----

def main(args=None):
    parser = argparse.ArgumentParser(description='Draw Sierpinski triangle with turtlesim.')
    parser.add_argument('--level', type=int, default=2, help='Recursion depth (e.g., 1–5)')
    parsed_args = parser.parse_args(args)

    speed_scale = 1.0 + (parsed_args.level - 1) * 0.5

    rclpy.init()
    turtle = TurtleController(speed_scale=speed_scale)

    # Initial triangle fits in 11x11 turtlesim window
    p1 = (2.5, 2.5)
    p2 = (8.5, 2.5)
    p3 = (5.5, 9.0)

    # Estimate how many triangles will be drawn: 3^level
    color_total = 3 ** parsed_args.level

    # Teleport to start without drawing
    turtle.set_pen(off=True)
    turtle.teleport_to(p1[0], p1[1])
    turtle.set_pen(r=0, g=0, b=255, width=2, off=False)

    # Draw recursive Sierpiński triangle
    sierpinski(turtle, p1, p2, p3, parsed_args.level, 0, parsed_args.level, 0, color_total)

    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])