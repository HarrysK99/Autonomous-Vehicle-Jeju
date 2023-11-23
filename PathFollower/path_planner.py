#!/usr/bin/env python3
import ray
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from PythonRobotics.PathPlanning.RRTStarReedsShepp import rrt_star_reeds_shepp

# Global variables for storing the map and goal
global_map = None
global_goal = [7,0]

def map_callback(msg):
    global global_map
    global_map = msg

def goal_callback(msg):
    global global_goal
    global_goal = msg

def create_path_message(rrt_path):
    path_msg = Path()
    path_msg.header.frame_id = "map"
    for point in rrt_path:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        path_msg.poses.append(pose)

    return path_msg

@ray.remote(num_cpus=4)
def plan(start, goal, obstacle_list, rand_area, max_iter):
    path_planner = rrt_star_reeds_shepp.RRTStarReedsShepp(start, goal, obstacle_list, rand_area, max_iter=max_iter)
    rrt_path = path_planner.planning(animation=False,search_until_max_iter=True)
    return rrt_path

def main():
    global global_map, global_goal
    rospy.init_node("rrt_star_reeds_shepp_planner_node")

    map_sub = rospy.Subscriber("/map", OccupancyGrid, map_callback)
    goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)
    path_pub = rospy.Publisher("/rrt_star_reeds_shepp_path", Path, queue_size=1)

    rate = rospy.Rate(1)

    ray.init()

    while not rospy.is_shutdown():
        if global_map is None and global_goal is not None:
            start = [0.0, 0.0,0.6]  # 테스트를 위한 초기 위치입니다. 로봇의 실제 위치로 교체해야 합니다.
            # goal = [global_goal.pose.position.x, global_goal.pose.position.y]  # 목표 위치를 기반으로한 RRT 목표입니다.
            goal=[global_goal[0], global_goal[1],0.6]

            obstacle_list = [[3,0.0,0.4],[5,0.2,0.2],[5.0,-0.2,0.2]]  # global_map을 기반으로 장애물 위치를 생성해야 합니다.

            line_points= [(i*0.1,-2,0.2) for i in range(0,70)] #원소 위치는 실제 거리.
            line_points+=[(i*0.1,2,0.2) for i in range(0,70)]

            obstacle_list+=line_points

            line_points+=[(3,2), (5,3),(5,1)]

            path_plans=[]
            for i in range(4):
                path_plans.append(plan.remote(start, goal, obstacle_list, [[0.0,7.0],[-2.0,2.0]], max_iter=20))
            
            # Gather results
            rrt_paths = ray.get(path_plans)

            # Get the best path
            best_path = None
            min_cost = float('inf')
            for path in rrt_paths:
                if path is not None:
                    cost = path[-1][2]
                    if cost < min_cost:
                        min_cost = cost
                        best_path = path
            
            rrt_path=best_path

            """
            path_planner = rrt_star_reeds_shepp.RRTStarReedsShepp(start, goal, obstacle_list, [[0.0,7.0],[-2.0,2.0]], max_iter=30)
            rrt_path = path_planner.planning(animation=False,search_until_max_iter=True)
            """

            if rrt_path:
                path_msg = create_path_message(rrt_path)
                path_pub.publish(path_msg)
            else:
                rospy.loginfo("No path found")

        rate.sleep()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass