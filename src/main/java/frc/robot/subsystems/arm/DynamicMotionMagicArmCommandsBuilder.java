package frc.robot.subsystems.arm;

import frc.utils.angle2d.Angle2d;
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
	public Command setTargetPosition(Angle2d position) {
		return arm
			.asSubsystemCommand(new InitExecuteCommand(() -> arm.setTargetPosition(position), () -> {}), "Set target position to: " + position);
	}

	public Command setTargetPosition(Supplier<Angle2d> position) {
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setTargetPosition(position.get())), "Set target position by supplier");
	}

	public Command setTargetPosition(Angle2d position, Angle2d maxVelocityAPS, Angle2d maxAccelerationAPSSquared) {
		return arm.asSubsystemCommand(
			new InitExecuteCommand(() -> {}, () -> arm.setTargetPosition(position, maxVelocityAPS, maxAccelerationAPSSquared)),
			"Set target position with "
				+ maxAccelerationAPSSquared
				+ " acceleration per Second Squared and with "
				+ maxVelocityAPS
				+ " velocity per second to:"
				+ position
		);
	}

}
