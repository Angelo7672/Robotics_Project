#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import cv2
from rospkg import RosPack


class Map:
    rospack = RosPack()
    folder = rospack.get_path('project2')
    width = 0
    height = 0
    resolution = 0.0
    pixel_origin_x = 0.0
    pixel_origin_y = 0.0
    pixel_xk0 = 0
    pixel_yk0 = 0
    first = True

def mapInfo(data):
    my_map = cv2.imread(map.folder + '/map/map.pgm')
    filename = map.folder + '/trajectory/trajectory.png'
    cv2.imwrite(filename, my_map)

    map.width = data.info.width
    map.height = data.info.height
    map.resolution = data.info.resolution
    map.pixel_origin_x = data.info.origin.position.x
    map.pixel_origin_y = data.info.origin.position.y
    trajectoryMaker()

def trajectoryMaker():
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callbackAmcl)
    rospy.spin()

def callbackAmcl(data):
    if map.first:
        map.first = False
        map.pixel_xk0 = int((data.pose.pose.position.x / map.resolution) - (map.pixel_origin_x / map.resolution))
        map.pixel_yk0 = -1 * int((data.pose.pose.position.y / map.resolution) - ((map.pixel_origin_y / map.resolution) + map.height))
        pixel_xk1 = map.pixel_xk0
        pixel_yk1 = map.pixel_yk0
    else:
        pixel_xk1 = int((data.pose.pose.position.x / map.resolution) - (map.pixel_origin_x / map.resolution))
        pixel_yk1 = -1 * int((data.pose.pose.position.y / map.resolution) - ((map.pixel_origin_y / map.resolution) + map.height))

    my_map = cv2.imread(map.folder + '/trajectory/trajectory.png')

    start_point = (map.pixel_xk0, map.pixel_yk0)
    end_point = (pixel_xk1, pixel_yk1)
    color = (0,255,0)
    thickness = 2
    trajectory = cv2.line(my_map, start_point, end_point, color, thickness)

    filename = map.folder + '/trajectory/trajectory.png'

    cv2.imwrite(filename, trajectory)

    map.pixel_xk0 = pixel_xk1
    map.pixel_yk0 = pixel_yk1

def listener():
    rospy.init_node('trajectory_saver', anonymous = False)

    rospy.Subscriber("/map", OccupancyGrid, mapInfo)

    rospy.spin()


if __name__ == '__main__':
    map = Map()
    listener()