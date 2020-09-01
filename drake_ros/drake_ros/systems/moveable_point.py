import geometry_msgs.msg
# from interactive_markers import InteractiveMarkerServer

# from tf2_ros import Buffer
# from tf3_ros import TransformListener

from pydrake.common.value import AbstractValue
from pydrake.systems.framework import LeafSystem
from pydrake.systems.framework import BasicVector_

from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import Marker


class MoveablePoint(LeafSystem):

    def __init__(self, server, tf_buffer, *, frame_id='WorldBody'):
        super().__init__()

        # TODO(sloretz) do I need to store allllll of this in the context?
        # if so, how tf do I get the context in callbacks?
        self._frame_id = frame_id
        self._tf_buffer = tf_buffer

        self._server = server
        server.insert(self._make_markers(), feedback_callback=self._feedback)
        server.applyChanges()

        # TODO(sloretz) constructor takes initial pose
        self._marker_point = [0.0, 0.0, 0.0]
        # TODO(sloretz) Drake systems would normally have this stuff in the context
        # but getting a context in callbacks does not seem straightforward
        # self._marker_state = self.DeclareDiscreteState(BasicVector_[float](3))

        self.DeclareVectorOutputPort(
            'point',
            BasicVector_[float](3),
            self._do_get_point)

    def _do_get_point(self, context, data):
        # marker_value = context.get_abstract_state(int(self._marker_state)).get_value()
        marker_value = self._marker_point
        data.SetFromVector(marker_value)

    def _feedback(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            # Only care about pose updates
            return

        if feedback.control_name not in ['move_x', 'move_y', 'move_z']:
            # Unexpected control name
            return

        point_stamped = geometry_msgs.msg.PointStamped()
        point_stamped.header = feedback.header
        point_stamped.point = feedback.pose.position

        if point_stamped.header.frame_id != self._frame_id:
            # TODO(sloretz) fix tf2_geometry_msgs_py :(
            # transformed_point = self._tf_buffer.transform(point_stamped, self._frame_id)
            print("TODO accept feedback in different frame")
            return

        # TODO(sloretz) this is where I would set the value on an output port in a drake system
        # TODO(sloretz) thread safety?
        self._marker_point[0] = point_stamped.point.x
        self._marker_point[1] = point_stamped.point.y
        self._marker_point[2] = point_stamped.point.z

    def _make_markers(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self._frame_id
        int_marker.name = 'golden_snitch'
        int_marker.description = 'tool tip target'

        # Create a sphere to mark where the tool tip should go
        snitch = Marker()
        snitch.type = Marker.SPHERE
        snitch.scale.x = 0.1
        snitch.scale.y = 0.1
        snitch.scale.z = 0.1
        snitch.color.r = 0.8
        snitch.color.g = 0.8
        snitch.color.b = 0.0
        snitch.color.a = 0.7

        snitch_control = InteractiveMarkerControl()
        snitch_control.always_visible = True
        snitch_control.markers.append(snitch)

        int_marker.controls.append(snitch_control)

        x_snitch_control = InteractiveMarkerControl()
        x_snitch_control.name = 'move_x'
        x_snitch_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        y_snitch_control = InteractiveMarkerControl()
        y_snitch_control.name = 'move_y'
        y_snitch_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        y_snitch_control.orientation.w = 0.7071068
        y_snitch_control.orientation.x = 0.0
        y_snitch_control.orientation.y = 0.0
        y_snitch_control.orientation.z = 0.7071068

        z_snitch_control = InteractiveMarkerControl()
        z_snitch_control.name = 'move_z'
        z_snitch_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        z_snitch_control.orientation.w = 0.7071068
        z_snitch_control.orientation.x = 0.0
        z_snitch_control.orientation.y = 0.7071068
        z_snitch_control.orientation.z = 0.0

        int_marker.controls.append(x_snitch_control)
        int_marker.controls.append(y_snitch_control)
        int_marker.controls.append(z_snitch_control)

        return int_marker

