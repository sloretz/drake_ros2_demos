import functools
import math

import geometry_msgs.msg
import numpy

from pydrake.common.eigen_geometry import AngleAxis
from pydrake.common.eigen_geometry import Quaternion
from pydrake.common.value import AbstractValue
from pydrake.math import RotationMatrix
from pydrake.multibody.tree import Joint_
from pydrake.multibody.tree import RevoluteJoint_
from pydrake.systems.framework import LeafSystem
from pydrake.systems.framework import BasicVector_

from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import Marker


class MoveableJoints(LeafSystem):

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
            print('Making stuff for', joint.name())
            if RevoluteJoint_.is_subclass_of_instantiation(type(joint)):
                server.insert(
                    self._make_revolute_marker(joint),
                    feedback_callback=functools.partial(self._revolute_feedback, joint))
            else:
                # TODO(sloretz) support more than revolute joints
                raise TypeError('joints must be an iterable of RevoluteJoint_[T]')
            # break  # xxx

        server.applyChanges()

        self.DeclareVectorOutputPort(
            'joint_states',
            BasicVector_[float](len(self._joint_states)),
            self._do_get_joint_states)

    def _revolute_feedback(self, joint, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return

        if feedback.control_name != f'rotate_axis_{joint.name()}':
            print(f'Unexpected control name {feedback.control_name} with joint {joint.name()}')
            # Unexpected control name
            return

        expected_frame = joint.child_body().body_frame().name()

        if expected_frame != feedback.header.frame_id:
            print(expected_frame, point_stamped.header.frame_id)
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

        # angle = 2.0 * math.atan2(math.sqrt(qx * qx + qy * qy + qz * qz), qw)
        angle_inc = diff_aa.angle()

        joint_axis = self._joint_axis_in_child[joint.name()]
        dot = numpy.dot(joint_axis, diff_aa.axis())
        print(joint_axis, diff_aa.axis())
        if dot > 0.999:
            angle_inc = diff_aa.angle()
        elif dot < -0.999:
            angle_inc = -1 * diff_aa.angle()
        else:
            angle_inc = 0

        # print('angle inc', angle)
        prev_angle = self._joint_states[self._joint_indexes[joint.name()]]
        angle = prev_angle + angle_inc

        if angle > joint.position_upper_limit():
            angle = joint.position_upper_limit()
            print('UPPER LIMIT', angle)
        elif angle < joint.position_lower_limit():
            angle = joint.position_lower_limit()
            print('LOWER LIMIT', angle)

        print('angle_inc', angle_inc, 'prev', prev_angle, 'angle', angle, 'dot', dot)

        self._joint_states[self._joint_indexes[joint.name()]] = angle
        self._joint_prev_orientation[joint.name()] = new_orientation

    def _do_get_joint_states(self, context, data):
        data.SetFromVector(self._joint_states)

    def _make_revolute_marker(self, revolute_joint: RevoluteJoint_):
        int_marker = InteractiveMarker()
        # int_marker.header.frame_id = revolute_joint.parent_body().body_frame().name()
        int_marker.header.frame_id = revolute_joint.child_body().body_frame().name()
        int_marker.name = revolute_joint.name()
        int_marker.scale = 0.3
        # print(f'marker name {int_marker.name}')

        # Drake revolute axis is in frame F on parent
        axis_hat = revolute_joint.revolute_axis()
        # F_to_parent_in_F = revolute_joint.frame_on_parent().GetFixedPoseInBodyFrame().inverse()
        # # Get parallel vector in parent frame
        # axis_in_parent = F_to_parent_in_F.rotation().multiply(axis_hat)
        # axis_in_parent_hat = axis_in_parent / numpy.linalg.norm(axis_in_parent)
        # # Store this info to help when calculating angle in feedback
        self._joint_axis_in_child[revolute_joint.name()] = axis_hat

        # What rotation would get the parent X axis to align with the joint axis?

        # what rotation would have gotten the x axis there?
        # https://math.stackexchange.com/q/476311
        # Find rotation matrix transforming X axis to Joint axis
        # Then convert that rotation matrix to quaternion for the interactive marker
        x_axis = (1, 0, 0)
        v = numpy.cross(x_axis, axis_hat)
        c = numpy.dot(x_axis, axis_hat)
        v_sub_x = numpy.array((
            (0, -v[2], v[1]),
            (v[2], 0, -v[0]),
            (-v[1], v[0], 0)), dtype=numpy.float64)
        v_sub_x_squared = numpy.dot(v_sub_x, v_sub_x)
        rotation_matrix = numpy.eye(3) + v_sub_x + v_sub_x_squared * (1.0 / (1.0 + c))
        pydrake_quat = RotationMatrix(rotation_matrix).ToQuaternion()
        # print("rotation_matrix", rotation_matrix)
        # print("axis_hat", axis_hat, numpy.linalg.norm(axis_hat))
        # print("axis_in_parent", axis_in_parent, numpy.linalg.norm(axis_in_parent))
        # print("axis_in_parent_hat", axis_in_parent_hat, numpy.linalg.norm(axis_in_parent_hat))
        # print("pydrake_quat", pydrake_quat)

        # joint_in_parent = revolute_joint.frame_on_parent().GetFixedPoseInBodyFrame()
        # x, y, z = joint_in_parent.translation()
        x, y, z = (0., 0., 0.)
        # quat = joint_in_parent.rotation().ToQuaternion()
        # print(quat)
        # joint_in_child = revolute_joint.frame_on_child().GetFixedPoseInBodyFrame()
        # print(joint_in_child.rotation().ToQuaternion())

        int_marker.pose.position.x = x
        int_marker.pose.position.y = y
        int_marker.pose.position.z = z

        #quat = quat.multiply(pydrake_quat)

        #int_marker.pose.orientation.w = quat.w()
        #int_marker.pose.orientation.x = quat.x()
        #int_marker.pose.orientation.y = quat.y()
        #int_marker.pose.orientation.z = quat.z()

        int_marker.pose.orientation.w = 1.
        int_marker.pose.orientation.x = 0.
        int_marker.pose.orientation.y = 0.
        int_marker.pose.orientation.z = 0.

        self._joint_prev_orientation[revolute_joint.name()] = pydrake_quat

        joint_control = InteractiveMarkerControl()
        joint_control.orientation.w = pydrake_quat.w()
        joint_control.orientation.x = pydrake_quat.x()
        joint_control.orientation.y = pydrake_quat.y()
        joint_control.orientation.z = pydrake_quat.z()

        joint_control.always_visible = True
        joint_control.name = f'rotate_axis_{revolute_joint.name()}'
        joint_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        # TODO (sloretz) Orientation needs to match joint axis
        int_marker.controls.append(joint_control)
        return int_marker
