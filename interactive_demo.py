#!/usr/bin/env python

import math
import os

from drake_ros.systems import MoveableJoints
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
# from pydrake.manipulation.planner import DifferentialInverseKinematicsIntegrator
from pydrake.manipulation.planner import DoDifferentialInverseKinematics
from pydrake.math import RigidTransform
from pydrake.math import RotationMatrix
# from pydrake.math import Quaternion
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import BodyIndex
from pydrake.multibody.tree import JointIndex
from pydrake.multibody.tree import WeldJoint
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import BasicVector_
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.framework import LeafSystem
from pydrake.systems.meshcat_visualizer import ConnectMeshcatVisualizer
from pydrake.systems.primitives import ConstantValueSource
from pydrake.systems.primitives import ConstantVectorSource
from pydrake.systems.primitives import Demultiplexer

import rclpy

from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String as StringMsg
from tf2_ros import Buffer
from tf2_ros import TransformListener
from tf2_ros import TransformBroadcaster


from pydrake.common import RandomDistribution
from pydrake.systems.primitives import RandomSource


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
        print('making ForwardKinematics with num_positions', num_positions)

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

        # print(joint_positions, self._plant.num_positions())
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


class TruncateVector(LeafSystem):
    """
    Ignore some inputs from a vector system.
    """

    def __init__(self, num_inputs, num_outputs):
        super().__init__()

        if num_outputs > num_inputs:
            raise ValueError('Must have more inputs than outputs')

        self._num_outputs = num_outputs

        self._input_port = self.DeclareVectorInputPort(
            'input', BasicVector_[float](num_inputs))

        self.DeclareVectorOutputPort(
            'output', BasicVector_[float](num_outputs), self._truncate_output)

    def _truncate_output(self, context, output):
        in_vec = self._input_port.Eval(context)
        output.SetFromVector(in_vec[:self._num_outputs])


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


class PadStartOfVector(LeafSystem):
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
        vector = [output] * self._difference
        vector = [v for v in in_vec]
        for v in in_vec:
            vector.append(v)
        output.SetFromVector(vector)


class MakeRigidTransform(LeafSystem):

    def __init__(self):
        super().__init__()

        self._position_input_port = self.DeclareVectorInputPort(
            'position', BasicVector_[float](3))

        self._orientation_input_port = self.DeclareVectorInputPort(
            'orientation', BasicVector_[float](4))

        self.DeclareAbstractOutputPort(
            'transform',
            lambda: AbstractValue.Make(RigidTransform()),
            self._do_get_transform)

    def _do_get_transform(self, context, data):
        position = self._position_input_port.Eval(context)
        orientation = self._orientation_input_port.Eval(context)

        rotation = RotationMatrix(Quaternion(orientation))
        data.set_value(RigidTransform(rotation, position))


# Copied from ManipulationStation example with some changes
#   added output port to say if it found a solution or not
#   got rid of the console spam
#   Made it take a RigidTransform instead of rpyxyz vector
class DifferentialIK(LeafSystem):
    def __init__(self, robot, frame_E, parameters, time_step):
        """
        @param robot is a reference to a MultibodyPlant.
        @param frame_E is a multibody::Frame on the robot.
        @param params is a DifferentialIKParams.
        @params time_step This system updates its state/outputs at discrete
                          periodic intervals defined with period @p time_step.
        """
        LeafSystem.__init__(self)
        self.robot = robot
        self.frame_E = frame_E
        self.parameters = parameters
        self.parameters.set_timestep(time_step)
        self.time_step = time_step
        # Note that this context is NOT the context of the DifferentialIK
        # system, but rather a context for the multibody plant that is used
        # to pass the configuration into the DifferentialInverseKinematics
        # methods.
        self.robot_context = robot.CreateDefaultContext()
        # Confirm that all velocities are zero (they will not be reset below).
        assert not self.robot.GetPositionsAndVelocities(
            self.robot_context)[-robot.num_velocities():].any()

        # Store the robot positions as state.
        self.DeclareDiscreteState(robot.num_positions())
        self.DeclarePeriodicDiscreteUpdate(time_step)

        # Desired pose of frame E in world frame.
        # self.DeclareInputPort("rpy_xyz_desired",
        #                       PortDataType.kVectorValued, 6)
        self._target_port = self.DeclareAbstractInputPort('target_pose',
            AbstractValue.Make(RigidTransform()))

        # Provide the output as desired positions.
        self.DeclareVectorOutputPort("joint_position_desired", BasicVector_[float](
            robot.num_positions()), self.CopyPositionOut)

        self._ik_state = self.DeclareAbstractState(AbstractValue.Make(True))
        self.DeclareAbstractOutputPort(
            'ik_working',
            lambda: AbstractValue.Make(bool()),
            self._is_ik_working)

    def _is_ik_working(self, context, data):
        ik_state = context.get_abstract_state(int(self._ik_state))
        data.set_value(ik_state.get_value())

    def SetPositions(self, context, q):
        context.get_mutable_discrete_state(0).SetFromVector(q)

    def ForwardKinematics(self, q):
        x = self.robot.GetMutablePositionsAndVelocities(
            self.robot_context)
        x[:self.robot.num_positions()] = q
        return self.robot.EvalBodyPoseInWorld(
            self.robot_context, self.frame_E.body())

    def CalcPoseError(self, X_WE_desired, q):
        pose = self.ForwardKinematics(q)
        err_vec = np.zeros(6)
        err_vec[-3:] = X_WE_desired.translation() - pose.translation()

        rot_err = AngleAxis(X_WE_desired.rotation()
                            * pose.rotation().transpose())
        err_vec[:3] = rot_err.axis() * rot_err.angle()

    def DoCalcDiscreteVariableUpdates(
            self, context, events, discrete_state):
        # rpy_xyz_desired = self.EvalVectorInput(context, 0).get_value()
        # X_WE_desired = RigidTransform(RollPitchYaw(rpy_xyz_desired[:3]),
        #                               rpy_xyz_desired[-3:]).GetAsIsometry3()
        X_WE_desired = self._target_port.Eval(context).GetAsIsometry3()
        q_last = context.get_discrete_state_vector().get_value()

        x = self.robot.GetMutablePositionsAndVelocities(
            self.robot_context)
        x[:self.robot.num_positions()] = q_last
        result = DoDifferentialInverseKinematics(self.robot,
                                                 self.robot_context,
                                                 X_WE_desired, self.frame_E,
                                                 self.parameters)

        ik_state = context.get_mutable_abstract_state(int(self._ik_state))

        if (result.status != result.status.kSolutionFound):
            discrete_state.get_mutable_vector().SetFromVector(q_last)
            ik_state.set_value(False)
        else:
            ik_state.set_value(True)
            discrete_state.get_mutable_vector().\
                SetFromVector(q_last + self.time_step*result.joint_velocities)

    def CopyPositionOut(self, context, output):
        output.SetFromVector(context.get_discrete_state_vector().get_value())


class VectorSelector(LeafSystem):
    """It's a multiplexer, but that name is already taken."""

    def __init__(self, data_type):
        super().__init__()
        self._false_port = self.DeclareVectorInputPort('false', data_type)
        self._true_port = self.DeclareVectorInputPort('true', data_type)

        self._selector_port = self.DeclareAbstractInputPort('select',
            AbstractValue.Make(bool()))

        self.DeclareVectorOutputPort(
            'output',
            data_type,
            self._do_get_output)

    def _do_get_output(self, context, data):
        if self._selector_port.Eval(context):
            selected_port = self._true_port
        else:
            selected_port = self._false_port
        data.SetFromVector(selected_port.Eval(context))


# class IsIKWorking(LeafSystem):
#     """Outputs True if Diff IK appears to have found a solution."""
# 
#     def __init__(self, num_joints, tolerance=0.001):
#         super().__init__()
# 
#         # Inputs
#         #   Joint positions commanded by DiffIK
#         # State
#         #   Last input from Diff IK
#         # Output
#         #   True if DiffIK appears to be moving end effector
#         self._command_port = self.DeclareVectorInputPort('position_command',
#             BasicVector_[float](num_joints))
# 
#         self._command_state = self.DeclareAbstractState(
#             AbstractValue.Make(BasicVector_[float](num_joints)))
# 
#         self.DeclareAbstractOutputPort(
#             'ik_working',
#             lambda: AbstractValue.Make(bool()),
#             self._do_get_output)
# 
#         self._tolerance = tolerance
# 
#     def _do_get_output(self, context, data):
#         state = context.get_mutable_abstract_state(int(self._command_state))
#         old_joint_positions = state.get_value()
#         current_joint_positions = self._command_port.Eval(context)
#         # print(type(old_joint_positions), type(current_joint_positions))
# 
#         if old_joint_positions.size() ==  0:
#             data.set_value(True)
#         elif current_joint_positions is not None:
#             # Not perfect, returns False when arm is at target pose
#             data_is_stale = True
#             for idx in range(min(old_joint_positions.size(), current_joint_positions.size)):
#                 old = old_joint_positions.GetAtIndex(idx)
#                 current = current_joint_positions[idx]
#                 if math.fabs(old - current) > self._tolerance:
#                     data_is_stale = False
#                     break
#             # print(f'old {ojp} new {cjp} stale {data_is_stale}')
#             data.set_value(not data_is_stale)
#         else:
#             data.set_value(False)
#         state.set_value(current_joint_positions)


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

    # robot = station.get_controller_plant()
    # diff_ik_parameters = DifferentialInverseKinematicsParameters(
    #     robot.num_positions(), robot.num_velocities())

    # time_step = 0.005
    # diff_ik_parameters.set_timestep(time_step)
    # iiwa14_velocity_limits = numpy.array([1.4, 1.4, 1.7, 1.3, 2.2, 2.3, 2.3])
    # diff_ik_parameters.set_joint_velocity_limits((-iiwa14_velocity_limits, iiwa14_velocity_limits))

    # integrator_sys = builder.AddSystem(DifferentialInverseKinematicsIntegrator(
    #     robot=robot,
    #     frame_E= robot.GetFrameByName("iiwa_link_7"),
    #     time_step=time_step,
    #     parameters=diff_ik_parameters,
    #     robot_context=robot.CreateDefaultContext()))

    # diff_ik_sys = builder.AddSystem(DifferentialIK(
    #     robot=robot,
    #     frame_E= robot.GetFrameByName("iiwa_link_7"),
    #     parameters=diff_ik_parameters,
    #     time_step=time_step))

    # selector_sys = builder.AddSystem(VectorSelector(BasicVector_[float](len(iiwa14_velocity_limits))))

    # home_joints_sys = builder.AddSystem(
    #     ConstantVectorSource(numpy.array([0.0] * len(iiwa14_velocity_limits))))

    # TODO(sloretz) system that chooses between based on diff ik failure
    # ik_or_home_sys = builder.AddSystem(IsIKWorking(len(iiwa14_velocity_limits)))
    # ConstantValueSource(AbstractValue.Make(False)))

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
    # target_system = builder.AddSystem(MoveablePoint(server, tf_buffer))
    joint_target_system = builder.AddSystem(MoveableJoints(server, tf_buffer, joints))

    fk_system = builder.AddSystem(ForwardKinematics(station.get_controller_plant()))

    print('built fk system')

    # Blindly assume first joints in plant are the iiwa14
    just_arm_joints = builder.AddSystem(
        PadVector(len(joints), station.get_controller_plant().num_joints(), 0.0))

    print('built just_arm_joints', station.get_controller_plant().num_joints(), len(joints))

    # const_quaternion_wxyz = builder.AddSystem(
    #     ConstantVectorSource(numpy.array([1.0 , 0.0, 0.0, 0.0]))
    # )

    # make_rigid_transform = builder.AddSystem(MakeRigidTransform())


    # TODO(sloretz) how to do I get body poses???
    # builder.Connect(
    #     station.get_multibody_plant().GetOutputPort('body_poses'),
    #     tf_system.GetInputPort('body_poses')
    # )

    builder.Connect(
        station.GetOutputPort('iiwa_position_measured'),
    #    just_arm_joints.get_input_port(0)
    #)

    #builder.Connect(
    #    just_arm_joints.get_output_port(0),
        fk_system.GetInputPort('joint_positions')
    )

    builder.Connect(
        fk_system.GetOutputPort('transforms'),
        tf_system.GetInputPort('body_poses')
    )
    # TODO how about system that gets joint states (from iiwa_position_measured),
    # then using context info for the station they use station.plant.CalcRelativeTransform()
    # to get poses between joint frames.
    # One, how do I get the frame names?
    #   From joints -> joint.frame_on_parent() and joint.frame_on_body
    # Two, how do I get the station's plant's context?
    #   
    # builder.Connect(
    #     station.GetOutputPort('iiwa_position_measured'),
    #     tf_system.GetInputPort('body_poses')
    # )
    builder.Connect(
        clock_system.GetOutputPort('clock'),
        tf_system.GetInputPort('clock')
    )

    # builder.Connect(
    #     target_system.GetOutputPort('point'),
    #     make_rigid_transform.GetInputPort('position')
    # )
    builder.Connect(
        joint_target_system.GetOutputPort('joint_states'),
        station.GetInputPort('iiwa_position')
    )

    # builder.Connect(
    #     const_quaternion_wxyz.get_output_port(0),
    #     make_rigid_transform.GetInputPort('orientation')
    # )
    #builder.Connect(
    #    make_rigid_transform.GetOutputPort('transform'),
    #    integrator_sys.GetInputPort('X_WE_desired')
    #)
    # builder.Connect(
    #     make_rigid_transform.GetOutputPort('transform'),
    #     diff_ik_sys.GetInputPort('target_pose')
    # )

    #builder.Connect(
    #    integrator_sys.GetOutputPort('joint_positions'),
    #    selector_sys.GetInputPort('true')
    #)
    # builder.Connect(
    #     diff_ik_sys.GetOutputPort('joint_position_desired'),
    #     selector_sys.GetInputPort('true')
    # )

    # builder.Connect(
    #     home_joints_sys.get_output_port(0),
    #     selector_sys.GetInputPort('false')
    # )

    # builder.Connect(
    #     integrator_sys.GetOutputPort('joint_positions'),
    #     ik_or_home_sys.get_input_port(0)
    # )

    # builder.Connect(
    #     ik_or_home_sys.get_output_port(0),
    #     selector_sys.GetInputPort('select')
    # )
    # builder.Connect(
    #     diff_ik_sys.GetOutputPort('ik_working'),
    #     selector_sys.GetInputPort('select')
    # )

    # builder.Connect(
    #     selector_sys.GetOutputPort('output'),
    #     station.GetInputPort('iiwa_position')
    # )

    # TODO(sloretz) why can't Drake Visualizer be connected to ManipulationStation's scene graph?
    # ConnectDrakeVisualizer(builder, station.get_scene_graph())
    ConnectDrakeVisualizer(builder, station.get_scene_graph(),
                           station.GetOutputPort("pose_bundle"))

    # visualizer = ConnectMeshcatVisualizer(
    #     builder,
    #     station.get_scene_graph(),
    #     station.GetOutputPort("pose_bundle"),
    #     zmq_url="new")

    constant_sys = builder.AddSystem(ConstantVectorSource(numpy.array([0.107])))
    builder.Connect(constant_sys.get_output_port(0),
                    station.GetInputPort("wsg_position"))

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator_context = simulator.get_mutable_context()
    simulator.set_target_realtime_rate(1.0)

    # print(diagram.GetGraphvizString())
    # input()

    # plant_context = diagram.GetSubsystemContext(plant, simulator_context)
    # integrator_context = diagram.GetSubsystemContext(integrator_sys, simulator_context)
    # integrator.SetPositions(integrator_context, plant.GetPositions(plant_context, model))
    station_context = station.GetMyMutableContextFromRoot(simulator_context)

    num_iiwa_joints = station.num_iiwa_joints()
    station.GetInputPort("iiwa_feedforward_torque").FixValue(
        station_context, numpy.zeros(num_iiwa_joints))

    # q0 = station.GetOutputPort("iiwa_position_measured").Eval(
    #     station_context)
    # integrator_sys.get_mutable_parameters().set_nominal_joint_position(q0)
    # integrator_context = integrator_sys.GetMyMutableContextFromRoot(simulator_context)
    # integrator_sys.SetPositions(integrator_context, q0)
    # diff_ik_context = diff_ik_sys.GetMyMutableContextFromRoot(simulator_context)
    # diff_ik_sys.SetPositions(diff_ik_context, q0)

    # import code
    # code.interact(local=locals())

    while simulator_context.get_time() < 12345:
        simulator.AdvanceTo(simulator_context.get_time() + 0.1)
        # TODO(sloretz) really need a spin_some in rclpy
        rclpy.spin_once(node, timeout_sec=0)

    rclpy.shutdown()
