#!/usr/bin/env python

import time

import numpy

from pydrake.common import FindResourceOrThrow
from pydrake.common.value import AbstractValue
from pydrake.geometry import ConnectDrakeVisualizer
from pydrake.geometry import FramePoseVector
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.systems.analysis import Simulator
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import Joint_
from pydrake.multibody.tree import JointIndex
from pydrake.multibody.tree import RevoluteJoint
from pydrake.multibody.tree import WeldJoint
from pydrake.systems.framework import BasicVector_
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.framework import LeafSystem
from pydrake.systems.framework import PublishEvent
from pydrake.systems.framework import TriggerType
from pydrake.systems.primitives import ConstantVectorSource

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
import rclpy
import rclpy.time
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String as StringMsg
from sensor_msgs.msg import JointState as JointStateMsg
from tf2_ros import TransformBroadcaster


# RuntimeError: Actuation input port for model instance iiwa must be connected.
def no_control(plant, builder, model):
    nu = plant.num_actuated_dofs(model)
    u0 = numpy.zeros(nu)
    constant = builder.AddSystem(ConstantVectorSource(u0))
    builder.Connect(
        constant.get_output_port(0),
        plant.get_actuation_input_port(model))


class SimulatorClock(LeafSystem):
    """Accumulates and outputs the time since the simulation started."""

    def __init__(self, *, publisher, period_sec):
        super().__init__()

        self.DeclareAbstractOutputPort(
            'clock',
            lambda: AbstractValue.Make(float),
            self._do_calculate_clock)

        self.DeclarePeriodicEvent(
            period_sec=period_sec,
            offset_sec=0.,
            event=PublishEvent(
                trigger_type=TriggerType.kPeriodic,
                callback=self._do_accumulate_clock))

        self._publisher = publisher
        self._period = period_sec
        self._time_state = self.DeclareAbstractState(AbstractValue.Make(float(0)))

    def _do_accumulate_clock(self, context, event):
        # Accumulate time from evenly spaced simulator ticks
        time_state = context.get_mutable_abstract_state(int(self._time_state))
        time_state.set_value(time_state.get_value() + self._period)

        # Publish simulated time for other ROS nodes
        time_msg = rclpy.time.Time(seconds=time_state.get_value()).to_msg()
        self._publisher.publish(time_msg)

    def _do_calculate_clock(self, context, data):
        time_value = context.get_abstract_state(int(self._time_state)).get_value()
        data.set_value(time_value)


class SystemClock(LeafSystem):
    """Outputs current system time without regard to simulator time."""

    def __init__(self):
        super().__init__()

        self.DeclareAbstractOutputPort(
            'clock',
            lambda: AbstractValue.Make(float),
            self._do_calculate_clock)

    def _do_calculate_clock(self, context, data):
        data.set_value(time.time())


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
        for joint in joints:
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
        tf_broadcaster.sendTransform(transforms)


class JointStatePublisher(LeafSystem):

    def __init__(self, *, publisher, joints, period_sec=1./60):
        super().__init__()

        # System will publish transforms for these joints only
        self._joints = tuple(joints)

        if 0 == len(self._joints):
            raise ValueError('Need at least one joint to publish joint states')

        for joint in self._joints:
            if not Joint_.is_subclass_of_instantiation(type(joint)):
                raise TypeError('joints must be an iterable of Joint_[T]')

        # This event publishes TF transforms at a regular interval
        self.DeclarePeriodicEvent(
            period_sec=period_sec,
            offset_sec=0.,
            event=PublishEvent(
                trigger_type=TriggerType.kPeriodic,
                callback=self._publish_joint_states))

    def _publish_joint_states(self, context, event):
        # TODO(sloretz) How to get plant context for calculating joint angle?
        # joint.get_angle(plant_context)
        pass


if __name__ == '__main__':
    sdf_file_path = FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/iiwa7/"
        "iiwa7_no_collision.sdf")
    print(sdf_file_path)

    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.01)

    parser = Parser(plant)

    model_name = "iiwa"
    model = parser.AddModelFromFile(sdf_file_path, model_name)
    # print(repr(model))
    # print(dir(plant))

    # Weld to world so it doesn't fall through floor :D
    base_frame = plant.GetFrameByName("iiwa_link_0", model)
    X_WB = RigidTransform([0, 0, 0])
    plant.WeldFrames(plant.world_frame(), base_frame, X_WB)

    plant.Finalize()

    # Must happen after Finalize
    # RuntimeError: Pre-finalize calls to 'num_actuated_dofs()' are not allowed; you must call Finalize() first.

    no_control(plant, builder, model)

    joints = []
    for i in range(plant.num_joints()):
        joints.append(plant.get_joint(JointIndex(i)))


    rclpy.init()
    node = rclpy.create_node('drake_demo')

    # Publish SDF content on robot_description topic
    latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    description_publisher = node.create_publisher(StringMsg, 'robot_description', qos_profile=latching_qos)

    # Plubish joint states on joint_states topic
    joint_states_publisher = node.create_publisher(JointStateMsg, 'joint_states', 1)

    # clock_publisher = node.create_publisher(TimeMsg, 'clock', 1)
    # clock_system = SimulatorClock(publisher=clock_publisher, period_sec=1./1000)

    clock_system = SystemClock()

    builder.AddSystem(clock_system)

    # Transform broadcaster for TF frames
    tf_broadcaster = TransformBroadcaster(node)

    # Connect to system that publishes TF transforms
    tf_system = TFPublisher(tf_broadcaster=tf_broadcaster, joints=joints)
    builder.AddSystem(tf_system)
    builder.Connect(
        plant.GetOutputPort('body_poses'),
        tf_system.GetInputPort('body_poses')
    )
    builder.Connect(
        clock_system.GetOutputPort('clock'),
        tf_system.GetInputPort('clock')
    )

    ConnectDrakeVisualizer(builder, scene_graph)

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator_context = simulator.get_mutable_context()
    simulator.set_target_realtime_rate(1.0)

    # Now that simulator is set up, connect to ROS

    msg = StringMsg()
    with open(sdf_file_path, 'r') as sdf_file:
        msg.data = sdf_file.read()

    description_publisher.publish(msg)
    # print(sdf_content)

    joints = []
    for i in range(plant.num_joints()):
        joints.append(plant.get_joint(JointIndex(i)))

    # import code
    # code.interact(local=locals())
    # raise Exception

    while simulator_context.get_time() < 12345:
        simulator.AdvanceTo(simulator_context.get_time() + 0.1)
        # TODO(sloretz) really need a spin_some in rclpy
        rclpy.spin_once(node, timeout_sec=0)

        # TODO(sloretz) publish clock topic

        joint_states = JointStateMsg()
        # TODO(sloretz) shouldn't this be simulated time?
        joint_states.header.stamp = node.get_clock().now().to_msg()

        diagram_context = diagram.CreateDefaultContext()
        plant_context = diagram.GetSubsystemContext(plant, diagram_context)
        # print(dir(plant_context))
        # print(plant.GetPositions(plant_context, model))

        # transforms = []

        # for joint in joints:
        #     # parent_frame = joint.frame_on_parent()
        #     # child_frame = joint.frame_on_child()

        #     # parent_frame = joint.parent_body().body_frame()
        #     # child_frame = joint.child_body().body_frame()

        #     parent_frame = plant.GetBodyByName(joint.parent_body().name()).body_frame()
        #     child_frame = plant.GetBodyByName(joint.child_body().name()).body_frame()

        #     # drake_tf = plant.CalcRelativeTransform(
        #     #     plant_context, parent_frame, child_frame)

        #     parent = joint.parent_body()
        #     child = joint.child_body()
        #     parent_pose = plant.EvalBodyPoseInWorld(plant_context, parent)
        #     child_pose = plant.EvalBodyPoseInWorld(plant_context, child)
        #     drake_tf = parent_pose.inverse().multiply(child_pose)

        #     # import code
        #     # code.interact(local=locals())
        #     # raise Exception

        #     # have transforms - will TF
        #     tf_stamped = TransformStamped()
        #     tf_stamped.header.stamp = node.get_clock().now().to_msg()
        #     tf_stamped.header.frame_id = parent_frame.name()
        #     tf_stamped.child_frame_id = child_frame.name()
        #     translation = tf_stamped.transform.translation
        #     translation.x, translation.y, translation.z = \
        #         drake_tf.translation()
        #     rotation = tf_stamped.transform.rotation
        #     rotation.w, rotation.x, rotation.y, rotation.z = \
        #         drake_tf.rotation().ToQuaternion().wxyz()
        #     # print(translation, rotation)
        #     transforms.append(tf_stamped)

        #     # Output joint states for those that find that useful
        #     if isinstance(joint, RevoluteJoint):
        #         # print(joint.name(), joint.get_angle(context))
        #         joint_states.name.append(joint.name())
        #         joint_states.position.append(joint.get_angle(plant_context))
        #     elif isinstance(joint, WeldJoint):
        #         # TODO should weld joints have static transforms?
        #         # Maybe not if one wants to re-run rviz regularly
        #         pass


        # if transforms:
        #     tf_broadcaster.sendTransform(transforms)
        # if joints:
        #     joint_states_publisher.publish(joint_states)
