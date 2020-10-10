#!/usr/bin/env python3

""" This is the starter code for the robot localization project """
import random

import rospy

from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan, PointCloud
from helper_functions import TFHelper
from occupancy_field import OccupancyField
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion

import tf
from tf import TransformListener
from tf import TransformBroadcaster

class Particle(object):
    def __init__(self,x=0.0,y=0.0,theta=0.0,w=1.0):
        # Initialize paticle values
        self.w = w
        self.x = x
        self.y = y
        self.theta = theta

    def as_pose(self):
        # COnvery to a pose!
        orientation_tuple = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        return Pose(position=Point(x=self.x, y=self.y, z=0),
                    orientation=Quaternion(x=orientation_tuple[0],
                                           y=orientation_tuple[1],
                                           z=orientation_tuple[2],
                                           w=orientation_tuple[3]))


class ParticleFilter(object):
    """ The class that represents a Particle Filter ROS Node
    """
    def __init__(self):
        rospy.init_node('pf')
        self.num_particles = 300
        self.particle_cloud = []
        self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)
        self.base_frame = "base_link"  # the frame of the robot base
        self.map_frame = "map"  # the name of the map coordinate frame
        self.odom_frame = "odom"  # the name of the odometry coordinate frame

        # pose_listener responds to selection of a new approximate robot
        # location (for instance using rviz)
        rospy.Subscriber("initialpose",
                         PoseWithCovarianceStamped,
                         self.update_initial_pose)

        # publisher for the particle cloud for visualizing in rviz.
        self.particle_pub = rospy.Publisher("particlecloud",
                                            PoseArray,
                                            queue_size=10)

        # create instances of two helper objects that are provided to you
        # as part of the project
        self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter
            based on a pose estimate.  These pose estimates could be generated
            by another ROS Node or could come from the rviz GUI """
        xy_theta = \
            self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)

        # initialize your particle filter based on the xy_theta tuple

        # Use the helper functions to fix the transform
    def create_particles(self, timestamp, xy_theta):
        for i in range(self.num_particles):
            # Add a new particle!!
            angle_variance = 10 # POint the points in the general direction of the robot

            x_cur = xy_theta[0]
            y_cur = xy_theta[1]
            theta_cur = xy_theta[2]
            x_rel = random.uniform(-.5, .5)
            y_rel = random.uniform(-.5, .5)
            new_theta = abs(random.randint(theta_cur-angle_variance, theta_cur+angle_variance) % 360)
            # TODO: Could use a tf transform to add x and y in the robot's coordinate system
            new_particle = Particle(x_cur+x_rel, y_cur+y_rel, new_theta)
            self.particle_cloud.append(new_particle)
            self.normalize_particles()


    def normalize_particles(self):
        total_weights = sum([particle.w for particle in self.particle_cloud])
        # if your weights aren't normalized then normalize them
        if total_weights != 1.0:
            for i in self.particle_cloud: i.w = i.w/total_weights

    def publish_particles(self):
        # Publish the particles so that one can see them in RVIZ
        # Convert the particles from xy_theta to pose!!
        pose_particle_cloud = []
        for p in self.particle_cloud:
            pose_particle_cloud.append(p.asPose())
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.map_frame),
                                  poses=pose_particle_cloud))

    def run(self):
        r = rospy.Rate(5)

        while not(rospy.is_shutdown()):
            print("running right file")
            # in the main loop all we do is continuously broadcast the latest
            # map to odom transform
            self.transform_helper.send_last_map_to_odom_transform()
            r.sleep()


if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
