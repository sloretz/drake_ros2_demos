from geometry_msgs.msg import TransformStamped

from pydrake.common.value import AbstractValue
from pydrake.math import RigidTransform
from pydrake.multibody.tree import Joint_
from pydrake.systems.framework import LeafSystem
from pydrake.systems.framework import PublishEvent
from pydrake.systems.framework import TriggerType

import rclpy.time
from tf2_ros import TransformBroadcaster


class TFPublisher(LeafSystem):

    def __init__(self, *, tf_broadcaster, joints, period_sec=1./60):
        super().__init__()

        if not isinstance(tf_broadcaster, TransformBroadcaster):
            raise TypeError('tf_broadcaster must be a TransformBroadcaster')
        self._tf_broadcaster = tf_broadcaster

        # System will publish transforms for these joints only
        self._joints = tuple(joints)

        if 0 == len(self._joints):
            raise ValueError('Need at least one joint to publish transforms')

        for joint in self._joints:
            if not Joint_.is_subclass_of_instantiation(type(joint)):
                raise TypeError('joints must be an iterable of Joint_[T]')

        self._time_input_port = self.DeclareAbstractInputPort(
            'clock', AbstractValue.Make(float))

        # This port receives body poses from MultibodyPlant
        self._poses_input_port = self.DeclareAbstractInputPort(
            'body_poses', AbstractValue.Make([RigidTransform()]))

        # This event publishes TF transforms at a regular interval
        self.DeclarePeriodicEvent(
            period_sec=period_sec,
            offset_sec=0.,
            event=PublishEvent(
                trigger_type=TriggerType.kPeriodic,
                callback=self._publish_tf))

    def _publish_tf(self, context, event):
        time_sec = self._time_input_port.Eval(context)
        body_poses = self._poses_input_port.Eval(context)

        ros_time = rclpy.time.Time(seconds=time_sec)

        transforms = []
        for joint in self._joints:
            # Get poses of bodies on both sides of the joint
            parent = joint.parent_body()
            child = joint.child_body()
            parent_pose = body_poses[int(parent.index())]
            child_pose = body_poses[int(child.index())]

            # Calculate the pose of child relative to parent
            drake_tf = parent_pose.inverse().multiply(child_pose)

            # Convert that into a TF transform ROS message
            tf_stamped = TransformStamped()
            tf_stamped.header.stamp = ros_time.to_msg()
            tf_stamped.header.frame_id = parent.body_frame().name()
            tf_stamped.child_frame_id = child.body_frame().name()
            translation = tf_stamped.transform.translation
            translation.x, translation.y, translation.z = \
                drake_tf.translation()
            rotation = tf_stamped.transform.rotation
            rotation.w, rotation.x, rotation.y, rotation.z = \
                drake_tf.rotation().ToQuaternion().wxyz()

            transforms.append(tf_stamped)

        # Publish transforms over a ROS topic!
        self._tf_broadcaster.sendTransform(transforms)
