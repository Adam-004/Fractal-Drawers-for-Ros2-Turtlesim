#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, TeleportAbsolute
import math
import asyncio
import random
from rclpy.executors import MultiThreadedExecutor

class PreciseKochTurtle(Node):
    def __init__(self):
        super().__init__('precise_koch_turtle')
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.speed = 1.5
        self.pos = [0.0, 0.0]
        self.heading = 0.0  # Radians

    async def set_pen(self, r, g, b, width=2, off=0):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        await self.pen_client.call_async(req)

    async def teleport(self, x, y, theta):
        await self.set_pen(0, 0, 0, off=1)
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        await self.teleport_client.call_async(req)
        await asyncio.sleep(0.1)
        self.pos = [x, y]
        self.heading = theta
        await self.set_pen(100, 200, 255, width=2)

    async def move_exact(self, dx, dy):
        target_angle = math.atan2(dy, dx)
        distance = math.sqrt(dx**2 + dy**2)
        angle_diff = (target_angle - self.heading + math.pi) % (2 * math.pi) - math.pi

        await self.turn(angle_diff)
        await self.set_pen(random.randint(100, 255), random.randint(100, 255), 255, 2)
        await self.move_forward(distance)

        self.heading = target_angle
        self.pos[0] += dx
        self.pos[1] += dy

    async def move_forward(self, distance):
        twist = Twist()
        twist.linear.x = self.speed
        self.vel_pub.publish(twist)
        await asyncio.sleep(distance / self.speed)
        self.stop()

    async def turn(self, angle):
        twist = Twist()
        twist.angular.z = self.speed if angle > 0 else -self.speed
        self.vel_pub.publish(twist)
        await asyncio.sleep(abs(angle) / self.speed)
        self.stop()

    def stop(self):
        self.vel_pub.publish(Twist())

    def compute_segment(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        dx = x2 - x1
        dy = y2 - y1

        a = (x1 + dx / 3, y1 + dy / 3)
        c = (x1 + 2 * dx / 3, y1 + 2 * dy / 3)

        angle = math.radians(60)
        mx = c[0] - a[0]
        my = c[1] - a[1]
        bx = math.cos(angle) * mx - math.sin(angle) * my + a[0]
        by = math.sin(angle) * mx + math.cos(angle) * my + a[1]
        b = (bx, by)

        return a, b, c

    async def draw_koch(self, p1, p2, depth):
        if depth == 0:
            dx = p2[0] - self.pos[0]
            dy = p2[1] - self.pos[1]
            await self.move_exact(dx, dy)
        else:
            a, b, c = self.compute_segment(p1, p2)
            await self.draw_koch(p1, a, depth - 1)
            await self.draw_koch(a, b, depth - 1)
            await self.draw_koch(b, c, depth - 1)
            await self.draw_koch(c, p2, depth - 1)

    async def draw_snowflake(self, vertices, depth):
        for i in range(3):
            p1 = vertices[i]
            p2 = vertices[(i + 1) % 3]
            heading = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
            await self.teleport(p1[0], p1[1], heading)
            await self.draw_koch(p1, p2, depth)

async def main_async():
    rclpy.init()
    node = PreciseKochTurtle()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    loop = asyncio.get_event_loop()
    loop.run_in_executor(None, executor.spin)

    node.pen_client.wait_for_service()
    node.teleport_client.wait_for_service()

    try:
        triangle = [(4.0, 4.0), (7.0, 4.0), (5.5, 6.8)]  # Neatly centered and scaled
        await node.draw_snowflake(triangle, depth=2)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

def main():
    asyncio.run(main_async())

if __name__ == '__main__':
    main()