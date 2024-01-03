#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg

class PoseStampedToTF:
    def __init__(self):
        # Initialize the node
        rospy.init_node('posestamped_to_tf_broadcaster')

        # Retrieve parameters
        self.parent_frame_id = rospy.get_param('~parent_frame_id', 'world')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'robot_base')

        # Create a TF2 broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        # Subscribe to PoseStamped topic
        rospy.Subscriber("/pose", geometry_msgs.msg.PoseStamped, self.handle_pose)

    def handle_pose(self, pose_stamped_msg):
        # Create a TransformStamped message
        t = geometry_msgs.msg.TransformStamped()

        # Fill in the fields of the message
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.parent_frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = pose_stamped_msg.pose.position.x
        t.transform.translation.y = pose_stamped_msg.pose.position.y
        t.transform.translation.z = pose_stamped_msg.pose.position.z
        t.transform.rotation.x = pose_stamped_msg.pose.orientation.x
        t.transform.rotation.y = pose_stamped_msg.pose.orientation.y
        t.transform.rotation.z = pose_stamped_msg.pose.orientation.z
        t.transform.rotation.w = pose_stamped_msg.pose.orientation.w

        # Broadcast the transform
        self.br.sendTransform(t)

if __name__ == '__main__':
    try:
        # Create the PoseStampedToTF object
        pstf = PoseStampedToTF()

        # Spin to keep the script for exiting
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
