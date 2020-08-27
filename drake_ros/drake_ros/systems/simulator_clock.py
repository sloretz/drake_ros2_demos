from pydrake.common.value import AbstractValue
from pydrake.systems.framework import LeafSystem
from pydrake.systems.framework import PublishEvent
from pydrake.systems.framework import TriggerType

import rclpy.time


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
