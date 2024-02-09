#!/usr/bin/env python3

# python linear algebra library
import numpy as np

# library for interacting with ROS and TF tree
import rospy
import tf2_ros

# ROS message types we need to use
from sensor_msgs.msg import NavSatFix, Imu

# SE3 library for handling poses and TF tree
from util.SE3 import SE3
from util.SO3 import SO3

class Localization:
    pose: SE3

    def __init__(self):
        # create subscribers for GPS and IMU data, linking them to our callback functions
        rospy.Subscriber("/gps/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/imu/imu_only", Imu, self.imu_callback)

        # create a transform broadcaster for publishing to the TF tree
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # initialize pose to all zeros
        self.pose = SE3()

    def gps_callback(self, msg: NavSatFix):
        """
        This function will be called every time this node receives a NavSatFix message
        on the /gps topic. It should read the GPS location from the given NavSatFix message,
        convert it to cartesian coordinates, store that value in `self.pose`, then publish
        that pose to the TF tree.
        """
        # rospy.loginfo(f"Received GPS data: {msg}")

        spherical_coord = np.array([msg.latitude, msg.longitude])
        cartesian_coord = self.spherical_to_cartesian(spherical_coord, [42.281, -83.743])

        # Update self.pose with the new position
        # Here we assume that the orientation has been previously set and is kept
        self.pose = SE3(position=cartesian_coord, rotation=self.pose.rotation)

        # Publish the updated pose to the TF tree
        self.pose.publish_to_tf_tree(self.tf_broadcaster, "map", "base_link")

    def imu_callback(self, msg: Imu):
        """
        This function will be called every time this node receives an Imu message
        on the /imu topic. It should read the orientation data from the given Imu message,
        store that value in `self.pose`, then publish that pose to the TF tree.
        """
        # rospy.loginfo(f"Received IMU data: {msg}")
        quaternion = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])

        # Update self.pose with the new orientation
        # Here we assume that the position has been previously set and is kept
        self.pose = SE3(position=self.pose.position, rotation=SO3(quaternion=quaternion))

        # Publish the updated pose to the TF tree
        self.pose.publish_to_tf_tree(self.tf_broadcaster, "map", "base_link")


    @staticmethod
    def spherical_to_cartesian(spherical_coord: np.ndarray, reference_coord: np.ndarray) -> np.ndarray:
        """
        This is a utility function that should convert spherical (latitude, longitude)
        coordinates into cartesian (x, y, z) coordinates using the specified reference point
        as the center of the tangent plane used for approximation.
        :param spherical_coord: the spherical coordinate to convert,
                                given as a numpy array [latitude, longitude]
        :param reference_coord: the reference coordinate to use for conversion,
                                given as a numpy array [latitude, longitude]
        :returns: the approximated cartesian coordinates in meters, given as a numpy array [x, y, z]
        """
        R = 6371000
        lat = np.radians(spherical_coord[0])
        lon = np.radians(spherical_coord[1])
        ref_lat = np.radians(reference_coord[0])
        ref_lon = np.radians(reference_coord[1])
        # Calculate the Cartesian coordinates before rotation
        x = R * (lat - ref_lat)
        y = R * (lon - ref_lon) * np.cos(ref_lat)
        # Rotate the point by 90 degrees to align with the simulator's reference heading
        # Rotation matrix for 90 degrees: [ [0, -1], [1, 0] ]
        x_rotated = -y
        y_rotated = x
        # Z coordinate is set to zero for this project
        z = 0
        return np.array([x_rotated, y_rotated, z])



def main():
    # initialize the node
    rospy.init_node("localization")

    # create and start our localization system
    localization = Localization()

    # let the callback functions run asynchronously in the background
    rospy.spin()


if __name__ == "__main__":
    main()
