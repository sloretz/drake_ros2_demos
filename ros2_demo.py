#!/usr/bin/env python

import os

import numpy

from drake_ros.systems import SystemClock
from drake_ros.systems import TFPublisher

from pydrake.geometry import ConnectDrakeVisualizer
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.systems.analysis import Simulator
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JointIndex
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource

import rclpy
import rclpy.time
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String as StringMsg
from tf2_ros import TransformBroadcaster


# RuntimeError: Actuation input port for model instance iiwa must be connected.
def no_control(plant, builder, model):
    nu = plant.num_actuated_dofs(model)
    u0 = numpy.zeros(nu)
    constant = builder.AddSystem(ConstantVectorSource(u0))
    builder.Connect(
        constant.get_output_port(0),
        plant.get_actuation_input_port(model))


if __name__ == '__main__':
    this_dir = os.path.abspath(os.path.dirname(__file__))

    sdf_file_path = os.path.join(this_dir, 'ur10.sdf')

    with open(os.path.join(this_dir, 'ur10.sdf.in'), 'r') as file_in:
        with open(sdf_file_path, 'w') as file_out:
            file_out.write(file_in.read().replace('PWD_GOES_HERE', this_dir))

    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)

    parser = Parser(plant)

    model_name = "ur10"
    model = parser.AddModelFromFile(sdf_file_path, model_name)

    # Weld to world so it doesn't fall through floor :D
    base_frame = plant.GetFrameByName("base", model)
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
    msg = StringMsg()
    with open(sdf_file_path, 'r') as sdf_file:
        msg.data = sdf_file.read()
    description_publisher.publish(msg)

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

    while simulator_context.get_time() < 12345:
        simulator.AdvanceTo(simulator_context.get_time() + 0.1)
        # TODO(sloretz) really need a spin_some in rclpy
        rclpy.spin_once(node, timeout_sec=0)
