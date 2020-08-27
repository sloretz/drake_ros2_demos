import time

from pydrake.common.value import AbstractValue
from pydrake.systems.framework import LeafSystem


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
