package frc.robot.turret.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotContainer;
import frc.robot.turret.TurretState;
import frc.robot.turret.TurretSubsystem;
import frc.utils.joysticks.SmartJoystick;

public abstract class TurretCommands {

    public static Command manualControl(SmartJoystick joystick) {
        return new FunctionalCommand(
                () -> RobotContainer.TURRET.setState(TurretState.MANUAL),
                () -> RobotContainer.TURRET.setPower(joystick.getAxisValue(SmartJoystick.Axis.RIGHT_X)),
                (interrupt) -> RobotContainer.TURRET.setState(TurretState.REST),
                () -> false,
                RobotContainer.TURRET
        ).withName("manual control");
    }

}
