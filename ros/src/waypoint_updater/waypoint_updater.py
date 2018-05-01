#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TLStatus

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

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/all_traffic_waypoint', TLStatus, self.traffic_state_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Add other member variables you need below
        self.max_lookahead = rospy.get_param('~max_lookahead', 200)
        self.acceleration = rospy.get_param('~acceleration', .5)
        self.deceleration = rospy.get_param('~deceleration', .5)
        self.base_lane = None
        self.pose = None
        self.velocity = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp = -1

        #use loop to get control over publisher frequency
        self.loop()

#    def loop(self):
        #run with 50Hz
#        rate=rospy.Rate(50)
#        while not rospy.is_shutdown():
#            if self.pose and self.base_waypoints:
#                #get closest waypoint
#                closest_waypoint_idx=self.get_closest_waypoint_idx()
#                #publish waypoints
#                self.publish_waypoints(closest_waypoint_idx)
#                rate.sleep()

    #function to get index of closest waypoint 
    def get_closest_waypoint_idx(self):
        x=self.pose.pose.position.x
        y=self.pose.pose.position.y
        closest_idx=self.waypoint_tree.query([x,y],1)[1]#0: position, 1: index
		
        #check if closest is in front or behind the own vehilce
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord=self.waypoints_2d[closest_idx-1]
		
        # equation for hyperplane through closest_coords
        cl_vec=np.arry(closest_coord)
        prev_vec=np.array(prev_coord)
        pos_vec=np.array([x,y])
        val=np.dot(cl_vec-prev_vec,pos_vec-cl_vec)
        #behind the car
        if (val>0):
            closest_idx=(closest_idx+1)%len(self.waypoints_2d)
            return closest_idx
		
    def publish_waypoints(self):
        lane = self.generate_lane()
        self.final_waypoints_pub.publish(lane)
        rospy.loginfo("sent a new lane")

    def generate_lane(self):
        lane = Lane()

        nn_idx = self.get_closest_waypoint_idx()
        look_ahead = nn_idx + self.max_lookahead
        base_wps = self.base_lane.waypoints[nn_idx : look_ahead]

        if self.stopline_wp == -1 or self.stopline_wp >= look_ahead:
            if self.velocity.linear.x / base_waypoints[0].twist.twist.linear.x < 0.5:
                lane.waypoints = accelerate(base_waypoints)
            else:
                lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate(base_waypoints, nn_idx)

        return lane

    def accelerate(self, waypoints):
        # TODO find destination for max speed
        # TODO calculate speed based on acceleration
        return waypoints

    def decelerate(self, waypoints, idx):
        speed_wps = []
        stop = max(self.stopline_wp - idx - 2, 0)
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            dist = self.distance(waypoints, i, stop)
            velocity = math.sqrt(2.0 * dist * self.deceleration)
            if 1.0 > velocity:
                velocity = 0.0

            p.twist.twist.linear.x = min(velocity, wp.twist.twist.linear.x)
            speed_wps.append(p)
        return speed_wps

    def pose_cb(self, msg):
        rospy.loginfo("received a pose")
        self.pose=msg
        self.publish_waypoints()

    def waypoints_cb(self, waypoints):
        rospy.loginfo("received waypoints")
        #store received waypoints in member
        self.base_waypoints = waypoints
        #use KDtree to find LOOKAHEAD_WPS
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x,waypoint.pose.pose.position.y] for waypoint in waypoints.waypoint]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        self.publish_waypoints()

    def traffic_cb(self, msg):
        rospy.loginfo("received a stopline")
        self.stopline_wp = msg
        self.publish_waypoints()

    def traffic_state_cb(self, tl_status):
        rospy.loginfo("received a traffic light state")
        if tl.status.state == TrafficLight.RED or tl_status.state == TrafficLight.YELLOW:
            self.stopline_wp = -1
        self.publish_waypoints()

    def velocity_cb(self, msg):
        rospy.loginfo("received a velocity")
        self.velocity = msg
        self.publish_waypoints()

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
