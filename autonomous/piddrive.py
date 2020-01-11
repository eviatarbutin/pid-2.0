from magicbot import StateMachine, state
from components.pidcontroller import PIDControl


class piddrive(StateMachine):
    MODE_NAME = "first pid"
    DEFAULT = True

    pidcontroller: PIDControl

    @state(first=True)
    def drive(self, initial_call):
        if initial_call:
            self.pidcontroller.setup_values(1, 2, 1, 0, 0.1, 0.02, 4, 15, 1, 2)
        elif self.pidcontroller.finished:
            self.done()
        else:
            self.pidcontroller.engage()

