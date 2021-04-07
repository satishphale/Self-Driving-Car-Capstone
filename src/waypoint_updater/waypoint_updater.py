#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np
from std_msgs.msg import Int32
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 
STOP_LINE_MARGIN = 3 
MAX_DECEL = 0.5 
CONSTANT_DECEL = 1 / LOOKAHEAD_WPS 

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        
        # TODO: Add other member variables you need below
        self.pose = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.base_lane = None
        self.stopline_wp_idx = -1

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        self.loop()

    def loop(self):
        rate = rospy.Rate(20) # 20 Hz 
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_wps()
            rate.sleep()
    
    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pose_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pose_vect - cl_vect)
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        
        return closest_idx
    
    def publish_wps(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)
    
    def generate_lane(self):
        lane = Lane()
        lane.header = self.base_lane.header

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_wps(base_waypoints, closest_idx)
        
        return lane

    def decelerate_wps(self, waypoints, closest_idx):
        temp = []

        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            stop_idx = max(self.stopline_wp_idx - closest_idx - STOP_LINE_MARGIN, 0)
            dist = self.distance(waypoints, i, stop_idx)

            velocity = math.sqrt(2 * MAX_DECEL * dist) + (i * CONSTANT_DECEL)
            if velocity < 1.:
                velocity = 0.
            
            p.twist.twist.linear.x = min(velocity, wp.twist.twist.linear.x)
            temp.append(p)
        
        return temp


    def pose_cb(self, message):
        self.pose = message

    def set_waypoint_velocity(self, wps, waypoint, vel):
        waypoints[waypoint].twist.twist.linear.x = vel

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
        

    def waypoints_cb(self, wps):
        self.base_lane = wps
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint 
                                in wps.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def get_waypoint_velocity(self, wp):
        return wp.twist.twist.linear.x



    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data






if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')