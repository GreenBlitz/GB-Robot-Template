package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.utilcommands.InitExecuteCommand;

public class DynamicMotionMagicArmCommandBuilder extends ArmCommandBuilder {

	private final DynamicMotionMagicArm arm;

	public DynamicMotionMagicArmCommandBuilder(DynamicMotionMagicArm arm) {
		super(arm);
		this.arm = arm;
	}

	@Override
	public Command setTargetPosition(Rotation2d position) {
		return arm
			.asSubsystemCommand(new InitExecuteCommand(() -> arm.setTargetPosition(position), () -> {}), "Set target position to: " + position);
	}

	public Command setTargetPosition(
		Rotation2d position,
		Rotation2d maxVelocityRotation2dPerSecond,
		Rotation2d maxAccelerationRotation2dPerSecondSquared,
		double arbitraryFeedForward
	) {
		return arm.asSubsystemCommand(
			new InitExecuteCommand(
				() -> arm.setTargetPosition(
					position,
					maxVelocityRotation2dPerSecond,
					maxAccelerationRotation2dPerSecondSquared,
					arbitraryFeedForward
				),
				() -> {}
			),
			"Set target position with "
				+ maxAccelerationRotation2dPerSecondSquared
				+ " acceleration per Second Squared and with "
				+ maxVelocityRotation2dPerSecond
				+ " velocity per second to:"
				+ position
		);
	}

}
