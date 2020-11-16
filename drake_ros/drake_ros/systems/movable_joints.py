import functools

import numpy

from pydrake.common.eigen_geometry import AngleAxis, Quaternion
from pydrake.math import RotationMatrix
from pydrake.multibody.tree import Joint_, RevoluteJoint_
from pydrake.systems.framework import BasicVector_, LeafSystem
from visualization_msgs.msg import (InteractiveMarker,
                                    InteractiveMarkerControl,
                                    InteractiveMarkerFeedback)


class MovableJoints(LeafSystem):

    def __init__(self, server, tf_buffer, joints):
        super().__init__()

        # TODO(sloretz) do I need to store allllll of this in the context?
        # if so, how tf do I get the context in callbacks?
        self._tf_buffer = tf_buffer

        # System will publish transforms for these joints only
        self._joints = tuple(joints)

        if 0 == len(self._joints):
            raise ValueError('Need at least one joint')

        for joint in self._joints:
            if not Joint_.is_subclass_of_instantiation(type(joint)):
                raise TypeError('joints must be an iterable of Joint_[T]')

        self._server = server

        self._joint_states = [0.0] * len(self._joints)
        # Map joint names to indexes in joint_states
        self._joint_indexes = {j.name(): i for j, i in zip(self._joints, range(len(self._joints)))}
        self._joint_prev_orientation = {j.name(): Quaternion() for j in self._joints}

        self._joint_axis_in_child = {}

        for joint in self._joints:
            if RevoluteJoint_.is_subclass_of_instantiation(type(joint)):
                server.insert(
                    self._make_revolute_marker(joint),
                    feedback_callback=functools.partial(self._revolute_feedback, joint))
            else:
                # TODO(sloretz) support more than revolute joints
                raise TypeError('joints must be an iterable of RevoluteJoint_[T]')

        server.applyChanges()

        self.DeclareVectorOutputPort(
            'joint_states',
            BasicVector_[float](len(self._joint_states)),
            self._do_get_joint_states)

    def _revolute_feedback(self, joint, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return

        expected_frame = joint.child_body().body_frame().name()

        if expected_frame != feedback.header.frame_id:
            # TODO(sloretz) fix tf2_geometry_msgs_py :(
            # transformed_point = self._tf_buffer.transform(point_stamped, self._frame_id)
            print("TODO accept feedback in different frame")
            return

        qw = feedback.pose.orientation.w
        qx = feedback.pose.orientation.x
        qy = feedback.pose.orientation.y
        qz = feedback.pose.orientation.z

        new_orientation = Quaternion(qw, qx, qy, qz)
        prev_orientation = self._joint_prev_orientation[joint.name()]
        orientation_diff = prev_orientation.inverse().multiply(new_orientation)
        diff_aa = AngleAxis(orientation_diff)

        joint_axis = self._joint_axis_in_child[joint.name()]
        dot = numpy.dot(joint_axis, diff_aa.axis())
        if dot > 0.999:
            angle_inc = diff_aa.angle()
        elif dot < -0.999:
            angle_inc = -1 * diff_aa.angle()
        else:
            angle_inc = 0.

        angle = self._joint_states[self._joint_indexes[joint.name()]] + angle_inc

        if angle > joint.position_upper_limit():
            angle = joint.position_upper_limit()
        elif angle < joint.position_lower_limit():
            angle = joint.position_lower_limit()

        self._joint_states[self._joint_indexes[joint.name()]] = angle
        self._joint_prev_orientation[joint.name()] = new_orientation

    def _do_get_joint_states(self, context, data):
        data.SetFromVector(self._joint_states)

    def _make_revolute_marker(self, revolute_joint: RevoluteJoint_):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = revolute_joint.child_body().body_frame().name()
        int_marker.name = revolute_joint.name()
        int_marker.scale = 0.3

        int_marker.pose.position.x = 0.
        int_marker.pose.position.y = 0.
        int_marker.pose.position.z = 0.
        int_marker.pose.orientation.w = 1.
        int_marker.pose.orientation.x = 0.
        int_marker.pose.orientation.y = 0.
        int_marker.pose.orientation.z = 0.

        # Drake revolute axis is in frame F on parent
        axis_hat = revolute_joint.revolute_axis()
        self._joint_axis_in_child[revolute_joint.name()] = axis_hat

        # What rotation would get the parent X axis to align with the joint axis?
        rotation_matrix = ComputeBasisFromAxis(0, axis_hat)
        pydrake_quat = RotationMatrix(rotation_matrix).ToQuaternion()

        joint_control = InteractiveMarkerControl()
        joint_control.orientation.w = pydrake_quat.w()
        joint_control.orientation.x = pydrake_quat.x()
        joint_control.orientation.y = pydrake_quat.y()
        joint_control.orientation.z = pydrake_quat.z()

        joint_control.always_visible = True
        joint_control.name = f'rotate_axis_{revolute_joint.name()}'
        joint_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        int_marker.controls.append(joint_control)
        return int_marker
