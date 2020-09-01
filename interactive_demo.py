#!/usr/bin/env python

import math
import os

from drake_ros.systems import MoveablePoint
from drake_ros.systems import SystemClock
from drake_ros.systems import TFPublisher

from interactive_markers import InteractiveMarkerServer

import numpy

from pydrake.common.eigen_geometry import Quaternion
from pydrake.common.value import AbstractValue
from pydrake.examples.manipulation_station import ManipulationStation
from pydrake.geometry import ConnectDrakeVisualizer
from pydrake.manipulation.planner import DifferentialInverseKinematicsParameters
from pydrake.manipulation.planner import DifferentialInverseKinematicsIntegrator
from pydrake.math import RigidTransform
from pydrake.math import RotationMatrix
# from pydrake.math import Quaternion
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JointIndex
from pydrake.multibody.tree import WeldJoint
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import BasicVector_
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.framework import LeafSystem
from pydrake.systems.meshcat_visualizer import ConnectMeshcatVisualizer
from pydrake.systems.primitives import ConstantValueSource
from pydrake.systems.primitives import ConstantVectorSource

import rclpy

from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String as StringMsg
from tf2_ros import Buffer
from tf2_ros import TransformListener
from tf2_ros import TransformBroadcaster


from pydrake.common import RandomDistribution
from pydrake.systems.primitives import RandomSource


class MakeRigidTransform(LeafSystem):

    def __init__(self):
        super().__init__()

        print('here19')
        self._position_input_port = self.DeclareVectorInputPort(
            'position', BasicVector_[float](3))

        print('here20')
        self._orientation_input_port = self.DeclareVectorInputPort(
            'orientation', BasicVector_[float](4))
        print('here21')

        self.DeclareAbstractOutputPort(
            'transform',
            lambda: AbstractValue.Make(RigidTransform()),
            self._do_get_transform)
        print('here18')

    def _do_get_transform(self, context, data):
        # print('here')
        position = self._position_input_port.Eval(context)
        # print('here1')
        orientation = self._orientation_input_port.Eval(context)
        # print('here2')

        rotation = RotationMatrix(Quaternion(orientation))
        data.set_value(RigidTransform(rotation, position))


# TODO system with actual control
# RuntimeError: Actuation input port for model instance iiwa must be connected.
def no_control(plant, builder, model):
    nu = plant.num_actuated_dofs(model)
    u0 = numpy.zeros(nu)
    constant = builder.AddSystem(ConstantVectorSource(u0))
    builder.Connect(
        constant.get_output_port(0),
        plant.get_actuation_input_port(model))


def random_control(plant, builder, model):
    num_dof = plant.num_actuated_dofs(model)
    distribution = RandomDistribution.kUniform
    sample_interval = 0.001
    random_sys = RandomSource(distribution, num_dof, sample_interval)
    constant = builder.AddSystem(random_sys)
    builder.Connect(
        constant.get_output_port(0),
        plant.get_actuation_input_port(model))


if __name__ == '__main__':
    # this_dir = os.path.abspath(os.path.dirname(__file__))

    # sdf_file_path = os.path.join(this_dir, 'ur10.sdf')

    # with open(os.path.join(this_dir, 'ur10.sdf.in'), 'r') as file_in:
    #     with open(sdf_file_path, 'w') as file_out:
    #         file_out.write(file_in.read().replace('PWD_GOES_HERE', this_dir))

    builder = DiagramBuilder()

    # plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)

    # parser = Parser(plant)

    # model_name = "ur10"
    # model = parser.AddModelFromFile(sdf_file_path, model_name)

    # Weld to world so it doesn't fall through floor :D
    # base_frame = plant.GetFrameByName("base", model)
    # X_WB = RigidTransform([0, 0, 0])
    # plant.WeldFrames(plant.world_frame(), base_frame, X_WB)

    # plant.Finalize()

    # Must happen after Finalize
    # RuntimeError: Pre-finalize calls to 'num_actuated_dofs()' are not allowed; you must call Finalize() first.
    # no_control(plant, builder, model)
    # random_control(plant, builder, model)

    # num_dof = plant.num_actuated_dofs(model)

    station = builder.AddSystem(ManipulationStation())
    station.SetupClutterClearingStation()
    station.Finalize()

    robot = station.get_controller_plant()
    diff_ik_parameters = DifferentialInverseKinematicsParameters(
        robot.num_positions(), robot.num_velocities())

    time_step = 0.005
    diff_ik_parameters.set_timestep(time_step)
    iiwa14_velocity_limits = numpy.array([1.4, 1.4, 1.7, 1.3, 2.2, 2.3, 2.3])
    diff_ik_parameters.set_joint_velocity_limits((-iiwa14_velocity_limits, iiwa14_velocity_limits))

    integrator = DifferentialInverseKinematicsIntegrator(
        robot=robot,
        frame_E= robot.GetFrameByName("iiwa_link_7"),
        time_step=time_step,
        parameters=diff_ik_parameters,
        robot_context=robot.CreateDefaultContext()
    )
    print('here12')

    builder.AddSystem(integrator)
    builder.Connect(
        integrator.GetOutputPort('joint_positions'),
        station.GetInputPort('iiwa_position')
        # plant.get_actuation_input_port(model)
    )
    print('here13')

    # joints = []
    # for i in range(robot.num_joints()):
    #     joints.append(robot.get_joint(JointIndex(i)))

    rclpy.init()
    node = rclpy.create_node('interactive_demo')

    print('here14')
    # Publish SDF content on robot_description topic
    # latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    # description_publisher = node.create_publisher(StringMsg, 'robot_description', qos_profile=latching_qos)
    # msg = StringMsg()
    # with open(sdf_file_path, 'r') as sdf_file:
    #     msg.data = sdf_file.read()
    # description_publisher.publish(msg)

    # clock_system = SystemClock()

    # builder.AddSystem(clock_system)

    # Transform broadcaster for TF frames
    # tf_broadcaster = TransformBroadcaster(node)

    # Connect to system that publishes TF transforms
    # tf_system = TFPublisher(tf_broadcaster=tf_broadcaster, joints=joints)
    # builder.AddSystem(tf_system)
    # builder.Connect(
    #     station.get_multibody_plant().GetOutputPort('body_poses'),
    #     tf_system.GetInputPort('body_poses')
    # )
    # builder.Connect(
    #     clock_system.GetOutputPort('clock'),
    #     tf_system.GetInputPort('clock')
    # )

    tf_buffer = Buffer(node=node)
    tf_listener = TransformListener(tf_buffer, node)
    server = InteractiveMarkerServer(node, 'tool_tip_target')
    target_system = builder.AddSystem(MoveablePoint(server, tf_buffer))

    print('here15')

    const_quaternion_wxyz = builder.AddSystem(
        ConstantVectorSource(numpy.array([1.0 , 0.0, 0.0, 0.0]))
    )

    make_rigid_transform = builder.AddSystem(MakeRigidTransform())
    print('here3')

    builder.Connect(
        target_system.GetOutputPort('point'),
        make_rigid_transform.GetInputPort('position')
    )
    print('here4')

    builder.Connect(
        const_quaternion_wxyz.get_output_port(0),
        make_rigid_transform.GetInputPort('orientation')
    )
    print('here5')
    builder.Connect(
        make_rigid_transform.GetOutputPort('transform'),
        integrator.GetInputPort('X_WE_desired')
    )
    print('here6')

    # TODO(sloretz) why can't Drake Visualizer be connected to ManipulationStation's scene graph?
    # ConnectDrakeVisualizer(builder, station.get_scene_graph())

    visualizer = ConnectMeshcatVisualizer(
        builder,
        station.get_scene_graph(),
        station.GetOutputPort("pose_bundle"),
        zmq_url="new")
    print('here7')

    constant_sys = builder.AddSystem(ConstantVectorSource(numpy.array([0.107])))
    builder.Connect(constant_sys.get_output_port(0),
                    station.GetInputPort("wsg_position"))
    print('here8')

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator_context = simulator.get_mutable_context()
    simulator.set_target_realtime_rate(1.0)
    print('here9')

    # plant_context = diagram.GetSubsystemContext(plant, simulator_context)
    integrator_context = diagram.GetSubsystemContext(integrator, simulator_context)
    # integrator.SetPositions(integrator_context, plant.GetPositions(plant_context, model))
    station_context = station.GetMyMutableContextFromRoot(simulator_context)

    print('here10')
    num_iiwa_joints = station.num_iiwa_joints()
    station.GetInputPort("iiwa_feedforward_torque").FixValue(
        station_context, numpy.zeros(num_iiwa_joints))

    print('here11')
    q0 = station.GetOutputPort("iiwa_position_measured").Eval(
        station_context)
    integrator.get_mutable_parameters().set_nominal_joint_position(q0)
    integrator_context = integrator.GetMyMutableContextFromRoot(simulator_context)
    integrator.SetPositions(integrator_context, q0)

    print('here12')
    while simulator_context.get_time() < 12345:
        simulator.AdvanceTo(simulator_context.get_time() + 0.1)
        # TODO(sloretz) really need a spin_some in rclpy
        rclpy.spin_once(node, timeout_sec=0)

    rclpy.shutdown()
