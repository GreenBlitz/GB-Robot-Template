package frc.robot.subsystems.motorSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class MotorCommandBuilder {

	private final MotorSubsystem motorSubsystem;

	public MotorCommandBuilder(MotorSubsystem motorSubsystem) {
		this.motorSubsystem = motorSubsystem;
	}

	public Command setTargetVelocity(Rotation2d velocity) {
		return new RunCommand(() -> motorSubsystem.setTargetVelocityRotation2dPerSecond(velocity), motorSubsystem);
	}

}
