package frc.robot.turret.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotContainer;
import frc.robot.turret.TurretState;
import frc.robot.turret.TurretSubsystem;
import frc.utils.joysticks.SmartJoystick;

import java.util.function.DoubleSupplier;

public abstract class TurretCommands {

    public static Command manualControl(DoubleSupplier powerSupplier) {
        return new FunctionalCommand(
                () -> RobotContainer.TURRET.setState(TurretState.MANUAL),
                () -> RobotContainer.TURRET.setPower(powerSupplier.getAsDouble()),
                (interrupt) -> RobotContainer.TURRET.setState(TurretState.REST),
                () -> false,
                RobotContainer.TURRET
        ).withName("manual control");
    }

    public static Command lookAtTarget(Translation2d target) {
        return new FunctionalCommand(
                () -> {
                    RobotContainer.TURRET.setState(TurretState.ROTATE_TO_POINT);
                    RobotContainer.TURRET.setTargetPoint(target);
                },
                () -> {},
                (interrupt) -> RobotContainer.TURRET.setState(TurretState.REST),
                () -> false,
                RobotContainer.TURRET
        ).withName("look at target");
    }

}
