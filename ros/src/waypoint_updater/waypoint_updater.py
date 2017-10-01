#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Point, TwistStamped
from styx_msgs.msg import Lane, Waypoint

import math

from waypoint_updater_logger import WaypointUpdaterLogger

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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
TARGET_CRUISE_V = 12
TARGET_CREEP_SPEED = 1.0
#TARGET_STOP_V = -8.0
TARGET_STOP_V = -16.0
#BRAKING_DISTANCE = 40.0
BRAKING_DISTANCE = 50.0
#HARD_STOP_DISTANCE = 10.0
HARD_STOP_DISTANCE = 30.0


class WaypointUpdater(object):
    def __init__(self):
        self.rate = 10
        self.track_waypoints = None
        self.car_point = None
        self.stop_waypoint_index = -1
        self.current_linear_velocity = 0.0
        self.breaking_velocities = None
        self.logger = WaypointUpdaterLogger(self, rate=1)

        rospy.init_node('waypoint_updater')

        self.bw_disposable = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop()

    def loop(self):
        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            if self.track_waypoints and self.car_point:
                self.compute_and_publish_final_waypoints()
            rate.sleep()

    def pose_cb(self, msg):
        self.car_point = msg.pose.position

    def waypoints_cb(self, waypoints):
        if self.track_waypoints is None:
            self.track_waypoints = waypoints
            self.bw_disposable.unregister()
            self.bw_disposable = None

    def traffic_cb(self, stop_waypoint_index):
        self.stop_waypoint_index = stop_waypoint_index.data

    def velocity_cb(self, message):
        self.current_linear_velocity = message.twist.linear.x

    def compute_and_publish_final_waypoints(self):
        waypoints = self.track_waypoints.waypoints
        next_waypoint_index = WaypointUpdater.next_waypoint(waypoints, self.car_point)

        should_break = False
        if self.stop_waypoint_index != -1 and self.stop_waypoint_index != next_waypoint_index:
            distance = WaypointUpdater.total_distance(waypoints, next_waypoint_index, self.stop_waypoint_index)
            should_break = distance < BRAKING_DISTANCE

        if should_break and not self.breaking_velocities:
            self.breaking_velocities = self.compute_breaking_velocities(
                next_waypoint_index,
                self.stop_waypoint_index)

        if not should_break:
            self.breaking_velocities = None

        lane = Lane()
        for j in range(next_waypoint_index, next_waypoint_index + LOOKAHEAD_WPS):
            waypoint = waypoints[j]
            target_linear_velocity = TARGET_CRUISE_V \
                if not should_break or j > self.stop_waypoint_index \
                else self.breaking_velocities[j]

            self.set_waypoint_velocity(waypoint, target_linear_velocity)
            lane.waypoints.append(waypoint)

        self.final_waypoints_pub.publish(lane)
        self.logger.log(next_waypoint_index, WaypointUpdater.total_distance)

    def compute_breaking_velocities(self, next_waypoint_next, stop_waypoint_index):
        current_v = self.current_linear_velocity
        breaking_velocities = {}

        for i in range(next_waypoint_next, stop_waypoint_index + 1):
            velocity = TARGET_CRUISE_V \
                if i < next_waypoint_next \
                else current_v + ((TARGET_STOP_V - current_v) *
                                  float(i - next_waypoint_next) / float(stop_waypoint_index - next_waypoint_next))

            if 0.0 < velocity < 1.0:
                velocity = TARGET_CREEP_SPEED
            elif velocity < 0.0:
                velocity = 1.0

            if WaypointUpdater.total_distance(self.track_waypoints.waypoints, i, stop_waypoint_index) < HARD_STOP_DISTANCE:
                velocity = 0.0

            breaking_velocities[i] = velocity

        return breaking_velocities

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity

    @staticmethod
    def distance(a, b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

    @staticmethod
    def total_distance(waypoints, wp1, wp2):
        dist = 0.0

        for i in range(wp1, wp2+1):
            dist += WaypointUpdater.distance(
                waypoints[wp1].pose.pose.position,
                waypoints[i].pose.pose.position)
            wp1 = i

        return dist

    @staticmethod
    def closest_waypoint(waypoints, car_point):
        next_waypoint_index = 0
        next_waypoint_dist = 99999

        for i, waypoint in enumerate(waypoints):
            dist = WaypointUpdater.distance(
                waypoint.pose.pose.position,
                car_point)

            if dist < next_waypoint_dist:
                next_waypoint_index = i
                next_waypoint_dist = dist

        return next_waypoint_index

    @staticmethod
    def next_waypoint(waypoints, car_point):
        next_waypoint_index = WaypointUpdater.closest_waypoint(waypoints, car_point)

        next1 = waypoints[next_waypoint_index].pose.pose.position
        next2 = waypoints[next_waypoint_index + 1].pose.pose.position

        theta = math.atan2(next2.y - next1.y, next2.x - next1.x)
        heading = math.atan2(next1.y - car_point.y, next1.x - car_point.x)
        angle = abs(theta - heading)

        if angle > math.pi / 4.0:
            next_waypoint_index += 1

        return next_waypoint_index

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
