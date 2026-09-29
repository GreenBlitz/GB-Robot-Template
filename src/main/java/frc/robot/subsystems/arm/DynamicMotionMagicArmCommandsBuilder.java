package frc.robot.subsystems.arm;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.command2.Command;
import org.wpilib.command2.RunCommand;
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
