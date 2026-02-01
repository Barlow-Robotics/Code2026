from commands2 import button, cmd
from subsystems import Shooter
from wpilib.simulation import JoystickSim


class TestController:
    """Small example to test controller bindings."""

    def __init__(self):
        self.count = 0

        self.trigger_cmd = cmd.runOnce(self.onTrigger)
        
    def onTrigger(self):
        self.count += 1
        print(f"TRIGGER {self.count}")


class Controller:
    def __init__(self, shooterSub: Shooter):
        self.shooterSub = shooterSub
        self.OPERATOR = button.CommandJoystick(0)
        self.DRIVER = button.CommandJoystick(1)

        self.test_controller = TestController()
        # self.listener = keyboard.Listener()
        # self.listener.start()
        self.joystick_sim = JoystickSim(0)  # Simulate joystick ID 0


    def setupTeleop(self):
        self.DRIVER.button(1).onTrue(self.test_controller.trigger_cmd)
        self.DRIVER.button(2).onTrue(self.shooterSub.set_velocity_command)
        # self.joystick_sim.axisGreaterThan(0, 0).onTrue(cmd.runOnce(self.shooterSub.set_velocity(1))).onFalse(cmd.runOnce(self.shooterSub.stop()))
        
        # def handle_simulated_input(self, key):
    #     """Handle keyboard input in simulation."""
    #     if key == "w":
    #         # Simulate pressing button 1 on the joystick
    #         self.joystick_sim.setRawButton(1, True)
    #     else:
    #         # Release the button for other keys
    #         self.joystick_sim.setRawButton(1, False)
    # def on_key_press(self, key):
    #     try:
    #         # Check if the 'W' key is pressed
    #         if key.char == 'w':
    #             # Run the command to set the shooter velocity
    #             cmd.runOnce(lambda: self.shooterSub.set_velocity(1)).schedule()
    #     except AttributeError:
    #         # Handle special keys (e.g., arrow keys, function keys)
    #         pass