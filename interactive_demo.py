#!/usr/bin/env python

import os

import numpy

import rclpy
from drake_ros.systems import MoveableJoints, SystemClock, TFPublisher
from interactive_markers import InteractiveMarkerServer
from pydrake.common.value import AbstractValue
from pydrake.examples.manipulation_station import ManipulationStation
from pydrake.geometry import ConnectDrakeVisualizer
from pydrake.math import RigidTransform
from pydrake.multibody.tree import BodyIndex
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import BasicVector_, DiagramBuilder, LeafSystem
from pydrake.systems.primitives import ConstantVectorSource
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String as StringMsg
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


class ForwardKinematics(LeafSystem):
    """
    Get forward kinematics of each joint on iiwa arm from manipulation station in world frame.

    Offers something like the MultibodyPlant body_poses port.
    """

    def __init__(self, plant):
        super().__init__()

        # A multibody plant with a robot added to it
        self._plant = plant
        # Drake needs context to go along with a class instance
        self._plant_context = self._plant.CreateDefaultContext()

        num_positions = self._plant.num_positions()

        # Input: List of joint positions matching order known by plant
        self._joint_positions_port = self.DeclareVectorInputPort(
            'joint_positions', BasicVector_[float](num_positions))
        # Output: Rigid transforms of each body in base frame
        self.DeclareAbstractOutputPort(
            'transforms',
            lambda: AbstractValue.Make([RigidTransform()]),
            self._do_forward_kinematics)

    def _do_forward_kinematics(self, context, data):
        joint_positions = self._joint_positions_port.Eval(context)

        # Set the latest positions in the plant context
        self._plant.SetPositions(self._plant_context, joint_positions)

        world_frame = self._plant.world_frame()

        transforms = []
        for i in range(self._plant.num_bodies()):
            body = self._plant.get_body(BodyIndex(i))

            # calculate pose of body in world frame
            body_frame = body.body_frame()
            transforms.append(
                self._plant.CalcRelativeTransform(
                    self._plant_context, world_frame, body_frame))

        data.set_value(transforms)


class PadVector(LeafSystem):
    """
    Add some constant values to pad a vector.
    """

    def __init__(self, num_inputs, num_outputs, output_value=0.0):
        super().__init__()

        if num_inputs > num_outputs:
            raise ValueError('Must have more outputs than inputs')

        self._difference = num_outputs - num_inputs
        self._output_value = output_value

        self._input_port = self.DeclareVectorInputPort(
            'input', BasicVector_[float](num_inputs))

        self.DeclareVectorOutputPort(
            'output', BasicVector_[float](num_outputs), self._pad_output)

    def _pad_output(self, context, output):
        in_vec = self._input_port.Eval(context)
        vector = [v for v in in_vec]
        for i in range(self._difference):
            vector.append(self._output_value)
        output.SetFromVector(vector)


def main():
    builder = DiagramBuilder()

    station = builder.AddSystem(ManipulationStation())
    station.SetupClutterClearingStation()
    station.Finalize()

    # Create robot description with re-written paths
    this_dir = os.path.abspath(os.path.dirname(__file__))
    sdf_file_path = os.path.join(this_dir, 'iiwa14_no_collision.sdf')
    with open(os.path.join(this_dir, 'iiwa14_no_collision.sdf.in'), 'r') as file_in:
        with open(sdf_file_path, 'w') as file_out:
            file_out.write(file_in.read().replace('PWD_GOES_HERE', this_dir))

    joint_names = [
        'iiwa_joint_1',
        'iiwa_joint_2',
        'iiwa_joint_3',
        'iiwa_joint_4',
        'iiwa_joint_5',
        'iiwa_joint_6',
        'iiwa_joint_7']
    joints = []
    for name in joint_names:
        joints.append(station.get_controller_plant().GetJointByName(name))

    rclpy.init()
    node = rclpy.create_node('interactive_demo')

    # Publish SDF content on robot_description topic
    latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    description_publisher = node.create_publisher(StringMsg, 'robot_description', qos_profile=latching_qos)
    msg = StringMsg()
    with open(sdf_file_path, 'r') as sdf_file:
        msg.data = sdf_file.read()
    description_publisher.publish(msg)

    clock_system = SystemClock()

    builder.AddSystem(clock_system)

    # Transform broadcaster for TF frames
    tf_broadcaster = TransformBroadcaster(node)

    # Connect to system that publishes TF transforms
    tf_system = builder.AddSystem(TFPublisher(tf_broadcaster=tf_broadcaster, joints=joints))
    tf_buffer = Buffer(node=node)
    tf_listener = TransformListener(tf_buffer, node)
    server = InteractiveMarkerServer(node, 'joint_targets')
    joint_target_system = builder.AddSystem(MoveableJoints(server, tf_buffer, joints))

    fk_system = builder.AddSystem(ForwardKinematics(station.get_controller_plant()))

    # Blindly assume first joints in plant are the iiwa14
    just_arm_joints = builder.AddSystem(
        PadVector(len(joints), station.get_controller_plant().num_joints(), 0.0))

    builder.Connect(
        station.GetOutputPort('iiwa_position_measured'),
        fk_system.GetInputPort('joint_positions')
    )

    builder.Connect(
        fk_system.GetOutputPort('transforms'),
        tf_system.GetInputPort('body_poses')
    )
    builder.Connect(
        clock_system.GetOutputPort('clock'),
        tf_system.GetInputPort('clock')
    )

    builder.Connect(
        joint_target_system.GetOutputPort('joint_states'),
        station.GetInputPort('iiwa_position')
    )
    ConnectDrakeVisualizer(builder, station.get_scene_graph(),
                           station.GetOutputPort("pose_bundle"))

    constant_sys = builder.AddSystem(ConstantVectorSource(numpy.array([0.107])))
    builder.Connect(constant_sys.get_output_port(0),
                    station.GetInputPort("wsg_position"))

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator_context = simulator.get_mutable_context()
    simulator.set_target_realtime_rate(1.0)

    station_context = station.GetMyMutableContextFromRoot(simulator_context)

    num_iiwa_joints = station.num_iiwa_joints()
    station.GetInputPort("iiwa_feedforward_torque").FixValue(
        station_context, numpy.zeros(num_iiwa_joints))

    while simulator_context.get_time() < 12345:
        simulator.AdvanceTo(simulator_context.get_time() + 0.01)
        # TODO(sloretz) really need a spin_some in rclpy
        rclpy.spin_once(node, timeout_sec=0)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
