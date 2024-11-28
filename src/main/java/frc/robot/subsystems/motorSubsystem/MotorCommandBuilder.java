package frc.robot.subsystems.motorSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class MotorCommandBuilder {

	private MotorSubsystem motor;

	public MotorCommandBuilder(MotorSubsystem motor) {
		this.motor = motor;
	}

	public Command setVelocity(Rotation2d velocity) {
		return new RunCommand(() -> motor.setTargetVelocityRotation2dPerSecond(velocity), motor);
	}

}
