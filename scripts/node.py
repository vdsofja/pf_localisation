#!/usr/bin/python3

"""
This is the main entry point for the particle filter exercise node. It
subscribes to laser, map, and odometry and creates an instance of
pf.PFLocaliser() to do the localisation.
"""

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy,ReliabilityPolicy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from pf_localisation.util import *

from geometry_msgs.msg import ( PoseStamped, PoseWithCovarianceStamped,
                                PoseArray, Quaternion,TransformStamped )
# from tf.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import pf_localisation.pf2 #todo change to pf
# from pf_localisation.wait_for_message import wait_for_message

import sys
from copy import deepcopy
from threading import Event

class ParticleFilterLocalisationNode(Node):
    def __init__(self):
        super().__init__('ParticleFilterLocalisationNode')
        # ----- Minimum change (m/radians) before publishing new particle cloud and pose
        # self._PUBLISH_DELTA = self.get_parameter_or("publish_delta",0.1) #.get_parameter_value().double_value
        self.action_done_event = False
        self._PUBLISH_DELTA = 0.1

        self._particle_filter = pf_localisation.pf2.PFLocaliser()

        self._latest_scan = None
        self._last_published_pose = None
        self._initial_pose_received = False

        self.callback_group = ReentrantCallbackGroup()
        self._pose_publisher = self.create_publisher(PoseStamped, "/estimatedpose", 10, callback_group=self.callback_group)
        self._amcl_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, "/amcl_pose",10, callback_group=self.callback_group )
        self._cloud_publisher = self.create_publisher(PoseArray, "/particlecloud", 10, callback_group=self.callback_group)
        self._tf_publisher = self.create_publisher(TransformStamped, "/tf", 10, callback_group=self.callback_group)

        self.get_logger().info("Waiting for a map...")
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._ocuccupancy_map_sub = self.create_subscription(OccupancyGrid, "/map", self._map_cb, qos_profile=latching_qos )
        self._map_timer = self.create_timer(20, self._timer_cb, callback_group=self.callback_group)


        qos_odom = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._laser_subscriber = self.create_subscription(LaserScan, "/base_scan", self._laser_callback,    qos_profile=qos_odom)
        self._initial_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self._initial_pose_callback,  qos_profile=qos_odom)
        self._odometry_subscriber = self.create_subscription(Odometry, "/odom",    self._odometry_callback, qos_profile=qos_odom)

    def _timer_cb(self):
        if not self.action_done_event:
            self.get_logger().error("Problem getting a map. Check that you have a map_server"
                     " running: rosrun map_server map_server <mapname> " )
            sys.exit(1)
        self._map_timer.destroy()

    def _map_cb(self, ocuccupancy_map: OccupancyGrid):
        if not self.action_done_event:
            self.action_done_event = True
            self.get_logger().info("Map received. %d X %d, %f px/m." %
                        (ocuccupancy_map.info.width, ocuccupancy_map.info.height,
                        ocuccupancy_map.info.resolution))
            self._particle_filter.set_map(ocuccupancy_map)


    def _initial_pose_callback(self, pose):
        """ called when RViz sends a user supplied initial pose estimate """
        self._particle_filter.set_initial_pose(pose)
        self._last_published_pose = deepcopy(self._particle_filter.estimatedpose)
        self._initial_pose_received = True
        self._cloud_publisher.publish(self._particle_filter.particlecloud)

    def _odometry_callback(self, odometry):
        """
        Odometry received. If the filter is initialised then execute
        a filter predict step with odeometry followed by an update step using
        the latest laser.
        """
        if self._initial_pose_received:
            t_odom = self._particle_filter.predict_from_odometry(odometry)
            self.get_logger().info("Odometry update: %fs"%t_odom)
            if self._latest_scan:
                t_filter = self._particle_filter.update_filter(self._latest_scan)
                self.get_logger().info("Particle update: %fs"%t_filter)
                if t_odom + t_filter > 0.1:
                    self.get_logger().warning("Filter cycle overran timeslot")

    def _laser_callback(self, scan):
        """
        Laser received. Store a ref to the latest scan. If robot has moved
        much, republish the latest pose to update RViz
        """
        self._latest_scan = scan
        if self._initial_pose_received:
            if  self._sufficientMovementDetected(self._particle_filter.estimatedpose):
                # ----- Publish the new pose
                self._amcl_pose_publisher.publish(self._particle_filter.estimatedpose)
                estimatedpose =  PoseStamped()
                estimatedpose.pose = self._particle_filter.estimatedpose.pose.pose
                estimatedpose.header.stamp = self._particle_filter.estimatedpose.header.stamp
                estimatedpose.header.frame_id = "map"
                self._pose_publisher.publish(estimatedpose)

                # ----- Update record of previously-published pose
                self._last_published_pose = deepcopy(self._particle_filter.estimatedpose)

                # ----- Get updated particle cloud and publish it
                self._cloud_publisher.publish(self._particle_filter.particlecloud)

                # ----- Get updated transform and publish it
                self._tf_publisher.publish(self._particle_filter.tf_message)

    def _sufficientMovementDetected(self, latest_pose):
        """
        Compares the last published pose to the current pose. Returns true
        if movement is more the self._PUBLISH_DELTA
        """
        # ----- Check that minimum required amount of movement has occurred before re-publishing
        latest_x = latest_pose.pose.pose.position.x
        latest_y = latest_pose.pose.pose.position.y
        prev_x = self._last_published_pose.pose.pose.position.x
        prev_y = self._last_published_pose.pose.pose.position.y
        location_delta = abs(latest_x - prev_x) + abs(latest_y - prev_y)

        # ----- Also check for difference in orientation: Take a zero-quaternion,
        # ----- rotate forward by latest_rot, and rotate back by prev_rot, to get difference)
        latest_rot = latest_pose.pose.pose.orientation
        prev_rot = self._last_published_pose.pose.pose.orientation

        q = rotateQuaternion(Quaternion(w=1.0),
                             getHeading(latest_rot))   # Rotate forward
        q = rotateQuaternion(q, -getHeading(prev_rot)) # Rotate backward
        heading_delta = abs(getHeading(q))
        #self.get_logger().info("Moved by %f"%location_delta)
        return (location_delta > self._PUBLISH_DELTA or
                heading_delta > self._PUBLISH_DELTA)


def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilterLocalisationNode()
    # executor = MultiThreadedExecutor()
    # executor.add_node(node)
    try:
        # executor = rclpy.executors.MultiThreadedExecutor()
        # executor.add_node(node)
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        rclpy.spin(node)
        # executor.spin()
    except KeyboardInterrupt:
         node.get_logger().info('Keyboard interrupt, shutting down.\n')
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)

    # finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()