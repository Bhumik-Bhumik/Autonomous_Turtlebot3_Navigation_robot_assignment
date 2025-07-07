#!/usr/bin/env python3

import rospy
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

path_pub = None

def world_to_grid(x, y, map_info):
    origin_x = map_info.origin.position.x
    origin_y = map_info.origin.position.y
    res = map_info.resolution
    gx = int((x - origin_x) / res)
    gy = int((y - origin_y) / res)
    return (gy, gx)

def grid_to_world(row, col, map_info):
    origin_x = map_info.origin.position.x
    origin_y = map_info.origin.position.y
    res = map_info.resolution
    x = col * res + origin_x + res / 2.0
    y = row * res + origin_y + res / 2.0
    return (x, y)

def heuristic(a, b):
    return np.hypot(b[0] - a[0], b[1] - a[1])

def astar(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, cost, current = heapq.heappop(open_set)

        if current == goal:
            break

        for neighbor in get_neighbors(current, grid):
            if grid[neighbor] >= 100:
                continue

            tentative_g = g_score[current] + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f, tentative_g, neighbor))
                came_from[neighbor] = current

    return reconstruct_path(came_from, start, goal)

def get_neighbors(cell, grid):
    r, c = cell
    neighbors = []
    for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
        nr, nc = r + dr, c + dc
        if 0 <= nr < grid.shape[0] and 0 <= nc < grid.shape[1]:
            neighbors.append((nr, nc))
    return neighbors

def reconstruct_path(parent, start, goal):
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = parent.get(current)
        if current is None:
            return []
    path.append(start)
    path.reverse()
    return path

def occupancy_callback(msg):
    global path_pub

    rospy.loginfo("Map received, computing A* path...")

    width = msg.info.width
    height = msg.info.height
    data = np.array(msg.data).reshape((height, width))

    start_world = (-2.0, 0.5)
    goal_world = (2.0, 0.0)

    start = world_to_grid(*start_world, msg.info)
    goal = world_to_grid(*goal_world, msg.info)

    if data[start] != 0 or data[goal] != 0:
        rospy.logwarn("Start or goal is not in free space.")
        return

    full_path = astar(data, start, goal)
    if not full_path:
        rospy.logwarn("No path found.")
        return

    # Reduce waypoints: sample every Nth cell
    N = 5
    reduced_path = full_path[::N]
    if full_path[-1] != reduced_path[-1]:
        reduced_path.append(full_path[-1])

    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = rospy.Time.now()

    for row, col in reduced_path:
        x, y = grid_to_world(row, col, msg.info)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        path_msg.poses.append(pose)

    rospy.loginfo("A* path published with %d points (downsampled)", len(path_msg.poses))
    path_pub.publish(path_msg)

def main():
    global path_pub
    rospy.init_node('global_planner_node')
    path_pub = rospy.Publisher('/planned_path', Path, queue_size=1, latch=True)
    rospy.Subscriber('/map', OccupancyGrid, occupancy_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

