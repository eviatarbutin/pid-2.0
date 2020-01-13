from components.pidcontroller import PIDControl
from magicbot import AutonomousStateMachine, state


class piddrive(AutonomousStateMachine):
    MODE_NAME = "ElevatorDriver"
    DEFAULT = True

    pid_controller: PIDControl

    @state(first=True)
    def drive(self, initial_call):
        if initial_call:
            self.pid_controller.setup_values()
        self.pid_controller.engage()
        if self.pid_controller.finished:
            self.done()
