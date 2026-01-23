from commands2 import button, cmd


class TestController:
    """Small example to test controller bindings."""

    def __init__(self):
        self.count = 0

        self.trigger_cmd = cmd.runOnce(self.onTrigger)

    def onTrigger(self):
        self.count += 1
        print(f"TRIGGER {self.count}")


class Controller:
    def __init__(self):
        self.OPERATOR = button.CommandJoystick(0)
        self.DRIVER = button.CommandJoystick(1)

        self.test_controller = TestController()

    def setupTeleop(self):
        self.DRIVER.button(1).onTrue(self.test_controller.trigger_cmd)
