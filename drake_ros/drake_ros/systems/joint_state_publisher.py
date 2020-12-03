from pydrake.multibody.tree import Joint_
from pydrake.systems.framework import LeafSystem, PublishEvent, TriggerType


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
