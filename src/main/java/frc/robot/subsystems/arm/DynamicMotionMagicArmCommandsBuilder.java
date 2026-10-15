package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.utils.utilcommands.InitExecuteCommand;

import java.util.function.Supplier;

public class DynamicMotionMagicArmCommandsBuilder extends ArmCommandsBuilder {

	private final DynamicMotionMagicArm arm;

	protected DynamicMotionMagicArmCommandsBuilder(DynamicMotionMagicArm arm) {
		super(arm);
		this.arm = arm;
	}

	@Override
	public Command setTargetPosition(Rotation2d position) {
		return arm
			.asSubsystemCommand(new InitExecuteCommand(() -> arm.setTargetPosition(position), () -> {}), "Set target position to: " + position);
	}

	public Command setTargetPosition(Supplier<Rotation2d> position) {
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setTargetPosition(position.get())), "Set target position by supplier");
	}

	public Command setTargetPosition(Rotation2d position, Rotation2d maxVelocityRPS, Rotation2d maxAccelerationRPSSquared) {
		return arm.asSubsystemCommand(
			new InitExecuteCommand(() -> {}, () -> arm.setTargetPosition(position, maxVelocityRPS, maxAccelerationRPSSquared)),
			"Set target position with "
				+ maxAccelerationRPSSquared
				+ " acceleration per Second Squared and with "
				+ maxVelocityRPS
				+ " velocity per second to:"
				+ position
		);
	}

}
