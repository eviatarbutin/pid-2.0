from components.pidcontroller import PIDControl
from magicbot import AutonomousStateMachine, state


class piddrive(AutonomousStateMachine):
    MODE_NAME = "ElevatorDriver"
    DEFAULT = True

    pid_controller: PIDControl

    @state(first=True)
    def drive(self):
        self.pid_controller.engage()
        if self.pid_controller.finished:
            self.done()
