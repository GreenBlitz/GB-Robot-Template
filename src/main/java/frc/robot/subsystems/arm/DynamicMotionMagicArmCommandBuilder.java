package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.Supplier;

public class DynamicMotionMagicArmCommandBuilder extends ArmCommandBuilder {

	private final DynamicMotionMagicArm arm;

	public DynamicMotionMagicArmCommandBuilder(DynamicMotionMagicArm arm) {
		super(arm);
		this.arm = arm;
	}

	@Override
	public Command setTargetPosition(Rotation2d position) {
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setTargetPosition(position)), "Set target position to: " + position);
	}

	public Command setTargetPosition(Supplier<Rotation2d> position) {
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setTargetPosition(position.get())), "Set target position to: " + position);
	}

	public Command setTargetPosition(
		Rotation2d position,
		Rotation2d maxVelocityRotation2dPerSecond,
		Rotation2d maxAccelerationRotation2dPerSecondSquared
	) {
		return arm.asSubsystemCommand(
			new RunCommand(() -> arm.setTargetPosition(position, maxVelocityRotation2dPerSecond, maxAccelerationRotation2dPerSecondSquared)),
			"Set target position with "
				+ maxAccelerationRotation2dPerSecondSquared
				+ " acceleration per Second Squared and with "
				+ maxVelocityRotation2dPerSecond
				+ " velocity per second to:"
				+ position
		);
	}

}
