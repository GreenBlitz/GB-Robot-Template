package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.utils.commands.InitExecuteCommand;

import java.util.function.DoubleSupplier;

public class SwerveCommands {

    private static final Swerve SWERVE = RobotContainer.SWERVE;

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed
     * open mode.
     *
     * @param xSupplier the target forwards power
     * @param ySupplier the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static Command getOpenLoopFieldRelativeDriveCommand(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier
    ) {
        return new InitExecuteCommand(
                () -> SWERVE.initializeDrive(false, DriveMode.NORMAL),
                () -> SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
    }

}
