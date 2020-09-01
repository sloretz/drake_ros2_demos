#!/usr/bin/env python

import numpy as np
# from IPython import get_ipython
# from ipywidgets import ToggleButton, ToggleButtons
from pydrake.all import (
    ConnectMeshcatVisualizer, DiagramBuilder,
    DifferentialInverseKinematicsParameters,
    DifferentialInverseKinematicsIntegrator,
    RigidTransform, RotationMatrix, Simulator
)

from pydrake.geometry import ConnectDrakeVisualizer
# from pydrake.systems.jupyter_widgets import PoseSliders, WidgetSystem
from pydrake.examples.manipulation_station import ManipulationStation, CreateClutterClearingYcbObjectList

from pydrake.common.value import AbstractValue
from pydrake.systems.primitives import ConstantValueSource
from pydrake.systems.primitives import ConstantVectorSource

builder = DiagramBuilder()

station = builder.AddSystem(ManipulationStation())

station.SetupClutterClearingStation()
#ycb_objects = CreateClutterClearingYcbObjectList()
#for model_file, X_WObject in ycb_objects:
#    station.AddManipulandFromFile(model_file, X_WObject)
station.AddManipulandFromFile(
    "drake/examples/manipulation_station/models/"
    + "061_foam_brick.sdf",
    RigidTransform(RotationMatrix.Identity(), [0, -0.6, 0.2]))
station.Finalize()

visualizer = ConnectMeshcatVisualizer(
    builder, 
    station.get_scene_graph(), 
    station.GetOutputPort("pose_bundle"),
    zmq_url="new")
#   server_args=server_args)
#   jupyter_comms=True)

# ConnectDrakeVisualizer(builder, station.get_scene_graph())

robot = station.get_controller_plant()
params = DifferentialInverseKinematicsParameters(robot.num_positions(),
                                                  robot.num_velocities())

time_step = 0.005
params.set_timestep(time_step)
# True velocity limits for the IIWA14 (in rad, rounded down to the first
# decimal)
iiwa14_velocity_limits = np.array([1.4, 1.4, 1.7, 1.3, 2.2, 2.3, 2.3])
params.set_joint_velocity_limits((-iiwa14_velocity_limits,
                                  iiwa14_velocity_limits))
differential_ik = builder.AddSystem(DifferentialInverseKinematicsIntegrator(
    robot, robot.GetFrameByName("iiwa_link_7"), time_step, params))
builder.Connect(differential_ik.get_output_port(),
                station.GetInputPort("iiwa_position"))

# teleop = builder.AddSystem(PoseSliders(
#     min_range = PoseSliders.MinRange(roll=0, pitch=-0.5, yaw=-np.pi, 
#                                      x=-0.6, y=-0.8, z=0.0),
#     max_range = PoseSliders.MaxRange(roll=2*np.pi, pitch=np.pi, yaw=np.pi,
#                                      x=0.8, y=0.3, z=1.1)
# ))
# builder.Connect(teleop.get_output_port(0), 
#                 differential_ik.get_input_port())
# wsg_buttons = ToggleButtons(value=0.107, description="SchunkWsg", 
#                             options=[('Open', 0.107), ('Close', 0.002)])
# wsg_teleop = builder.AddSystem(WidgetSystem([wsg_buttons]))
# builder.Connect(wsg_teleop.get_output_port(0),
#                 station.GetInputPort("wsg_position"))

import inspect
print(inspect.getsourcefile(ConnectMeshcatVisualizer))

const_target = builder.AddSystem(
    ConstantValueSource(AbstractValue.Make(RigidTransform([0.2, 0.2, 0.2])))
)
builder.Connect(const_target.get_output_port(0), 
                differential_ik.get_input_port())
constant_sys = builder.AddSystem(ConstantVectorSource(np.array([0.107])))
builder.Connect(constant_sys.get_output_port(0),
                station.GetInputPort("wsg_position"))

diagram = builder.Build()
simulator = Simulator(diagram)
context = simulator.get_mutable_context()

station_context = station.GetMyMutableContextFromRoot(context)

num_iiwa_joints = station.num_iiwa_joints()
station.GetInputPort("iiwa_feedforward_torque").FixValue(
    station_context, np.zeros(num_iiwa_joints))

q0 = station.GetOutputPort("iiwa_position_measured").Eval(
    station_context)
differential_ik.get_mutable_parameters().set_nominal_joint_position(q0)
diff_ik_context = differential_ik.GetMyMutableContextFromRoot(context)
differential_ik.SetPositions(diff_ik_context, q0)
# teleop.SetPose(differential_ik.ForwardKinematics(diff_ik_context))

# if get_ipython():  # Then we're not just running as a test on CI.
#     simulator.set_target_realtime_rate(1.0)
# 
#     # Open the meshcat visualizer window (check your pop-up blocker).
#     open_window(visualizer.vis.url(), "meshcat")
# 
#     stop_button = ToggleButton(value=False, description='Stop Simulation')
#     display(stop_button)
#     while not stop_button.value:
#         simulator.AdvanceTo(simulator.get_context().get_time() + 5.0)
#     stop_button.value = False

simulator.set_target_realtime_rate(1.0)
# open_window(visualizer.vis.url(), "meshcat")
while True:
    simulator.AdvanceTo(simulator.get_context().get_time() + 5.0)
