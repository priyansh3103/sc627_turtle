#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
import numpy as np
import math
import heapq
from collections import deque
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class PlannerServer(Node):

    def __init__(self):
        super().__init__('planner_server')

        # QoS for latched map
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map_inflated',
            self.map_callback,
            map_qos
        )

        path_qos = QoSProfile(depth=1)
        path_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.astar_pub = self.create_publisher(Path, '/path_astar', path_qos)
        self.gvd_pub   = self.create_publisher(Path, '/path_gvd', path_qos)
        self.marker_pub = self.create_publisher(Marker,'/planner_markers',10)

        self.follow_path_client = ActionClient(self, FollowPath, '/follow_path')

        self.map_received = False
        self.get_logger().info("Planner server started")


    def map_callback(self, msg):
        if self.map_received:
            return
        self.map_received = True

        # Map metadata
        self.res = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height

        # Occupancy grid
        grid = np.array(msg.data, dtype=np.int8).reshape(self.height, self.width)
        grid[grid != 0] = 1   # 1 = occupied, 0 = free

        # Start and goal
        start_world = (0.0, 0.0)
        goal_world  = (-1.2, 0.44)

        start = self.world_to_grid(start_world)
        goal  = self.world_to_grid(goal_world)

        self.get_logger().info(f"Start grid: {start}, value={grid[start[1]][start[0]]}")
        self.get_logger().info(f"Goal  grid: {goal}, value={grid[goal[1]][goal[0]]}")
        self.get_logger().info(
    f"Grid value at start: {grid[start[1]][start[0]]}, "
    f"goal: {grid[goal[1]][goal[0]]}"
)

        # Publish start (green)
        self.publish_point_marker(
            start_world[0],
            start_world[1],
            marker_id=0,
            r=0.0, g=1.0, b=0.0
        )

        # Publish goal (red)
        self.publish_point_marker(
            goal_world[0],
            goal_world[1],
            marker_id=1,
            r=1.0, g=0.0, b=0.0
        )

        # A*
        astar_path = self.astar(grid, start, goal)

        # GVD (Brushfire + biased A*)
        clearance = self.brushfire(grid)
        gvd_path = self.gvd_astar(grid, clearance, start, goal)

        if astar_path:
            self.publish_path(astar_path, self.astar_pub)
            self.get_logger().info("A* path published")
            self.send_follow_path(astar_path)

        if gvd_path:
            self.publish_path(gvd_path, self.gvd_pub)
            self.get_logger().info("GVD path published")


    def world_to_grid(self, p):
        x = int((p[0] - self.origin_x) / self.res)
        y = int((p[1] - self.origin_y) / self.res)
        return (x, y)


    def neighbors(self, x, y):
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),
                       (-1,-1),(-1,1),(1,-1),(1,1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                yield nx, ny


    def astar(self, grid, start, goal):
        h = lambda a, b: math.hypot(a[0]-b[0], a[1]-b[1])

        open_list = []
        heapq.heappush(open_list, (0, start))

        came = {}
        g = {start: 0}
        closed = set()

        while open_list:
            _, curr = heapq.heappop(open_list)

            if curr in closed:
                continue
            closed.add(curr)

            if curr == goal:
                return self.reconstruct(came, curr)

            for n in self.neighbors(*curr):
                if grid[n[1]][n[0]] != 0 or n in closed:
                    continue

                cost = g[curr] + h(curr, n)
                if n not in g or cost < g[n]:
                    g[n] = cost
                    came[n] = curr
                    heapq.heappush(open_list, (cost + h(n, goal), n))

        return None


    def brushfire(self, grid):
        dist = np.full(grid.shape, np.inf)
        q = deque()

        for y in range(self.height):
            for x in range(self.width):
                if grid[y][x] == 1:
                    dist[y][x] = 0
                    q.append((x, y))

        while q:
            x, y = q.popleft()
            for nx, ny in self.neighbors(x, y):
                if dist[ny][nx] > dist[y][x] + 1:
                    dist[ny][nx] = dist[y][x] + 1
                    q.append((nx, ny))

        return dist


    def gvd_astar(self, grid, clearance, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start))

        came = {}
        g = {start: 0}
        closed = set()

        while open_list:
            _, curr = heapq.heappop(open_list)

            if curr in closed:
                continue
            closed.add(curr)

            if curr == goal:
                return self.reconstruct(came, curr)

            for n in self.neighbors(*curr):
                if grid[n[1]][n[0]] != 0 or n in closed:
                    continue

                length_cost = math.hypot(curr[0]-n[0], curr[1]-n[1])
                clearance_bonus = min(clearance[n[1]][n[0]], 10.0)

                cost = g[curr] + length_cost - 0.05 * clearance_bonus

                if n not in g or cost < g[n]:
                    g[n] = cost
                    came[n] = curr
                    f = cost + math.hypot(n[0]-goal[0], n[1]-goal[1])
                    heapq.heappush(open_list, (f, n))

        return None


    def reconstruct(self, came, curr):
        path = [curr]
        while curr in came:
            curr = came[curr]
            path.append(curr)
        return path[::-1]


    def publish_path(self, grid_path, pub):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in grid_path:
            p = PoseStamped()
            p.header.frame_id = "map"              # ← REQUIRED
            p.header.stamp = msg.header.stamp      # ← REQUIRED

            p.pose.position.x = self.origin_x + (x + 0.5) * self.res
            p.pose.position.y = self.origin_y + (y + 0.5) * self.res
            p.pose.position.z = 0.05               # lift above map
            p.pose.orientation.w = 1.0

            msg.poses.append(p)

        pub.publish(msg)

    def send_follow_path(self, grid_path):
        """Send the A* path to Nav2 controller via FollowPath action."""
        if not self.follow_path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 follow_path action server not available')
            return

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in grid_path:
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = path_msg.header.stamp
            p.pose.position.x = self.origin_x + (x + 0.5) * self.res
            p.pose.position.y = self.origin_y + (y + 0.5) * self.res
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            path_msg.poses.append(p)

        goal = FollowPath.Goal()
        goal.path = path_msg
        self.get_logger().info('Sending A* path to Nav2 controller')
        future = self.follow_path_client.send_goal_async(
            goal, feedback_callback=self._follow_feedback_cb)
        future.add_done_callback(self._follow_response_cb)

    def _follow_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 follow_path goal rejected')
            return
        self.get_logger().info('Nav2 follow_path goal accepted')
        goal_handle.get_result_async().add_done_callback(self._follow_result_cb)

    def _follow_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        if hasattr(fb, 'distance_to_goal'):
            self.get_logger().info(f'Distance to goal: {fb.distance_to_goal:.2f} m')

    def _follow_result_cb(self, future):
        status = future.result().status
        self.get_logger().info(f'Nav2 follow_path finished with status {status}')

    def publish_point_marker(self, x, y, marker_id, r, g, b):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "planner"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

def main():
    rclpy.init()
    node = PlannerServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()