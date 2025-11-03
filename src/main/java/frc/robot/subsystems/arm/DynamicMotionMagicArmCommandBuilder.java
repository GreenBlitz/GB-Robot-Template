package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.utils.math.ToleranceMath;
import frc.utils.utilcommands.InitExecuteCommand;

import java.util.function.Supplier;

public class DynamicMotionMagicArmCommandBuilder extends ArmCommandBuilder {

    private final DynamicMotionMagicArm arm;

    public DynamicMotionMagicArmCommandBuilder(DynamicMotionMagicArm arm) {
        super(arm);
        this.arm = arm;
    }

    public Command moveToPosition(Rotation2d position) {
        return arm
                .asSubsystemCommand(new InitExecuteCommand(() -> arm.withPosition(position), () -> {}), "Set target position to: " + position);
    }


    public Command moveToPosition(
            Rotation2d position,
            Rotation2d maxVelocityRotation2dPerSecond,
            Rotation2d maxAccelerationRotation2dPerSecondSquared,
            double arbitraryFeedForward
    ) {
        return arm.asSubsystemCommand(
                new InitExecuteCommand(
                        () -> arm.withPosition(
                                position,
                                maxVelocityRotation2dPerSecond,
                                maxAccelerationRotation2dPerSecondSquared,
                                arbitraryFeedForward
                        ),
                        () -> {}
                ),
                "Set target position to: " + position
        );
    }

}
