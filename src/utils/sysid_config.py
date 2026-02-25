from phoenix6.hardware import TalonFX
from phoenix6 import controls
from commands2.sysid import SysIdRoutine


def generateSysIdProfile(self, motor_spindex: TalonFX, name: str = "Spindex"):
    voltage_request = controls.VoltageOut(0)
    sys_id_routine = SysIdRoutine(
        SysIdRoutine.Config(),
        SysIdRoutine.Mechanism(
            lambda vol: motor_spindex.set_control(voltage_request.with_output(vol)),
            lambda log: (
                log.motor(name)
                .voltage(motor_spindex.get_motor_voltage().value_as_double)
                .position(motor_spindex.get_position().value_as_double)
                .velocity(motor_spindex.get_velocity().value_as_double)
            ),
            self,
        ),
    )
    return sys_id_routine
